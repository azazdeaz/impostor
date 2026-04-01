from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np
import newton
import warp as wp
from newton import ParticleFlags
from newton.math import quat_between_vectors_robust

from .mesh.mesh_builder import LeafBlueprint, StemBlueprint


@dataclass
class ClothBinding:
    """Particle-to-body binding data for cloth leaves."""
    bind_particle_ids: list[int] = field(default_factory=list)
    bind_body_ids: list[int] = field(default_factory=list)

    def merge(self, other: ClothBinding) -> ClothBinding:
        return ClothBinding(
            bind_particle_ids=self.bind_particle_ids + other.bind_particle_ids,
            bind_body_ids=self.bind_body_ids + other.bind_body_ids,
        )


@dataclass
class SkinBinding:
    """Precomputed barycentric mapping from visual mesh vertices to control points.

    Control points can be sourced from either cloth particles
    (``cp_particle_ids >= 0``) or rigid body transforms
    (``cp_body_ids >= 0`` with ``cp_local_offsets``).
    At runtime, ``gather_skin_controls`` populates a flat control-positions
    array and ``interpolate_skin_positions`` interpolates visual vertex
    positions via barycentric coordinates.
    """
    visual_indices: np.ndarray      # (M, 3) int32 — visual triangle indices
    num_visual_verts: int
    num_control_points: int
    cp_particle_ids: np.ndarray     # (num_cp,) int32 — particle ID or -1
    cp_body_ids: np.ndarray         # (num_cp,) int32 — body ID or -1
    cp_local_offsets: np.ndarray    # (num_cp, 3) float32
    tri_indices: np.ndarray         # (T, 3) int32 — collider tri indices into control pts
    face_ids: np.ndarray            # (num_visual_verts,) int32
    bary_u: np.ndarray              # (num_visual_verts,) float32
    bary_v: np.ndarray              # (num_visual_verts,) float32

    def merge(self, other: SkinBinding) -> SkinBinding:
        """Concatenate two bindings, offsetting indices so they don't collide."""
        vi_offset = self.num_visual_verts
        cp_offset = self.num_control_points
        tri_offset = len(self.tri_indices)
        return SkinBinding(
            visual_indices=np.vstack([self.visual_indices, other.visual_indices + vi_offset]),
            num_visual_verts=self.num_visual_verts + other.num_visual_verts,
            num_control_points=self.num_control_points + other.num_control_points,
            cp_particle_ids=np.concatenate([self.cp_particle_ids, other.cp_particle_ids]),
            cp_body_ids=np.concatenate([self.cp_body_ids, other.cp_body_ids]),
            cp_local_offsets=np.vstack([self.cp_local_offsets, other.cp_local_offsets]),
            tri_indices=np.vstack([self.tri_indices, other.tri_indices + cp_offset]),
            face_ids=np.concatenate([self.face_ids, other.face_ids + tri_offset]),
            bary_u=np.concatenate([self.bary_u, other.bary_u]),
            bary_v=np.concatenate([self.bary_v, other.bary_v]),
        )


@dataclass
class NewtonModelResult:
    """Result of build_newton_model: builder + cloth binding data."""
    builder: newton.ModelBuilder
    cloth_bindings: ClothBinding = field(default_factory=ClothBinding)
    skin: SkinBinding | None = None


@dataclass
class _LeafClothInfo:
    """Internal: tracks graph-node indices for a single leaf."""
    blueprint: LeafBlueprint
    midrib_node_indices: list[int] = field(default_factory=list)
    vein_node_indices: list[list[int]] = field(default_factory=list)


@dataclass
class _StemVisualInfo:
    """Internal: tracks graph-node indices for a stem with a mesh context."""
    blueprint: StemBlueprint
    node_indices: list[int] = field(default_factory=list)


@wp.kernel
def bind_particles_to_bodies(
    body_q: wp.array(dtype=wp.transform),
    bind_body_ids: wp.array(dtype=wp.int32),
    bind_particle_ids: wp.array(dtype=wp.int32),
    local_offsets: wp.array(dtype=wp.vec3),
    particle_q_0: wp.array(dtype=wp.vec3),
    particle_q_1: wp.array(dtype=wp.vec3),
):
    """Pin cloth particles to rigid bodies each substep.

    Uses precomputed body-local offsets to move particles that sit along
    the leaf skeleton (midrib/vein nodes) so they track the rod bodies.
    Both state buffers are written so the solver sees consistent positions.
    """
    tid = wp.tid()
    body_idx = bind_body_ids[tid]
    p_idx = bind_particle_ids[tid]
    xform = body_q[body_idx]
    # Transform the body-local offset into world space
    world_pos = wp.transform_point(xform, local_offsets[tid])
    particle_q_0[p_idx] = world_pos
    particle_q_1[p_idx] = world_pos


@wp.kernel
def gather_skin_controls(
    particle_q: wp.array(dtype=wp.vec3),
    body_q: wp.array(dtype=wp.transform),
    cp_particle_ids: wp.array(dtype=wp.int32),
    cp_body_ids: wp.array(dtype=wp.int32),
    cp_local_offsets: wp.array(dtype=wp.vec3),
    out_positions: wp.array(dtype=wp.vec3),
):
    """Resolve control point positions from mixed sources (step 1 of skinning).

    Each control point is either:
      - a cloth particle (cp_particle_ids >= 0) → read directly from particle_q
      - a rigid body point (cp_body_ids >= 0)   → transform local offset by body_q
    The result is a flat array of world-space positions used by
    interpolate_skin_positions.
    """
    tid = wp.tid()
    pid = cp_particle_ids[tid]
    if pid >= 0:
        # Leaf cloth: control point is a simulated particle
        out_positions[tid] = particle_q[pid]
    else:
        # Stem rod: control point is rigidly attached to a body
        bid = cp_body_ids[tid]
        xform = body_q[bid]
        out_positions[tid] = wp.transform_point(xform, cp_local_offsets[tid])


@wp.kernel
def interpolate_skin_positions(
    control_positions: wp.array(dtype=wp.vec3),
    tri_indices: wp.array2d(dtype=wp.int32),
    face_ids: wp.array(dtype=wp.int32),
    bary_u: wp.array(dtype=wp.float32),
    bary_v: wp.array(dtype=wp.float32),
    out_positions: wp.array(dtype=wp.vec3),
):
    """Interpolate visual vertex positions via barycentric coords (step 2 of skinning).

    Each visual vertex was mapped to a collider triangle (face_ids) with
    precomputed barycentric weights (bary_u, bary_v). The collider triangle
    vertices are looked up from control_positions (populated by
    gather_skin_controls).
    """
    tid = wp.tid()
    fi = face_ids[tid]
    u = bary_u[tid]
    v = bary_v[tid]
    # Fetch the three collider triangle vertices
    a = control_positions[tri_indices[fi, 0]]
    b = control_positions[tri_indices[fi, 1]]
    c = control_positions[tri_indices[fi, 2]]
    # Barycentric interpolation: P = (1-u-v)*A + u*B + v*C
    out_positions[tid] = (1.0 - u - v) * a + u * b + v * c


def _closest_point_on_triangle(
    p: np.ndarray, a: np.ndarray, b: np.ndarray, c: np.ndarray
) -> tuple[float, float]:
    """Return barycentric (u, v) of the closest point on triangle (a,b,c) to p.

    Convention: point = (1-u-v)*a + u*b + v*c.
    Uses Voronoi region tests to handle vertex, edge, and interior cases.
    """
    ab = b - a
    ac = c - a
    ap = p - a
    d1 = ab @ ap
    d2 = ac @ ap
    if d1 <= 0.0 and d2 <= 0.0:
        return 0.0, 0.0

    bp_ = p - b
    d3 = ab @ bp_
    d4 = ac @ bp_
    if d3 >= 0.0 and d4 <= d3:
        return 1.0, 0.0

    cp_ = p - c
    d5 = ab @ cp_
    d6 = ac @ cp_
    if d6 >= 0.0 and d5 <= d6:
        return 0.0, 1.0

    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        v = d1 / (d1 - d3)
        return v, 0.0

    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        w = d2 / (d2 - d6)
        return 0.0, w

    va = d3 * d6 - d5 * d4
    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return 1.0 - w, w

    denom = 1.0 / (va + vb + vc)
    v = vb * denom
    w = vc * denom
    return v, w


def _compute_barycentric_mapping(
    visual_verts: np.ndarray,
    collider_verts: np.ndarray,
    collider_tris: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """For each visual vertex, find the closest collider triangle + barycentric coords.

    Returns (face_ids, bary_u, bary_v) arrays.
    """
    n = len(visual_verts)
    face_ids = np.empty(n, dtype=np.int32)
    bary_u = np.empty(n, dtype=np.float32)
    bary_v = np.empty(n, dtype=np.float32)

    # Precompute triangle vertex positions for all collider faces
    tri_a = collider_verts[collider_tris[:, 0]]
    tri_b = collider_verts[collider_tris[:, 1]]
    tri_c = collider_verts[collider_tris[:, 2]]

    # Brute-force: for each visual vertex, test all collider triangles
    for i, p in enumerate(visual_verts):
        best_dist = float("inf")
        best_fi = 0
        best_u = 0.0
        best_v = 0.0
        for fi in range(len(collider_tris)):
            u, v = _closest_point_on_triangle(p, tri_a[fi], tri_b[fi], tri_c[fi])
            proj = (1.0 - u - v) * tri_a[fi] + u * tri_b[fi] + v * tri_c[fi]
            d = float(np.linalg.norm(p - proj))
            if d < best_dist:
                best_dist = d
                best_fi = fi
                best_u = u
                best_v = v
        face_ids[i] = best_fi
        bary_u[i] = best_u
        bary_v[i] = best_v

    return face_ids, bary_u, bary_v


def _to_wp(pos) -> wp.vec3:
    return wp.vec3(float(pos[0]), float(pos[1]), float(pos[2]))


def _collect_graph(
    blueprint: StemBlueprint | LeafBlueprint,
    nodes: list[wp.vec3],
    edges: list[tuple[int, int]],
    scales: list[float],
    parent_node_idx: int | None,
    include_stems: bool,
    include_midrib: bool,
    include_veins: bool,
    leaf_infos: list[_LeafClothInfo] | None = None,
    stem_infos: list[_StemVisualInfo] | None = None,
) -> None:
    """Recursively collect nodes and edges from the blueprint tree.

    Emits raw nodes/edges/scales without deduplication — call _deduplicate_nodes after.
    If *leaf_infos* is provided, appends a _LeafClothInfo for each leaf encountered.
    If *stem_infos* is provided, appends a _StemVisualInfo for each stem with a mesh_context.
    """
    if isinstance(blueprint, StemBlueprint):
        if not include_stems:
            return
        transforms = blueprint.transforms
        if not transforms:
            for child in blueprint.nodes:
                _collect_graph(child.blueprint, nodes, edges, scales,
                               parent_node_idx,
                               include_stems, include_midrib, include_veins,
                               leaf_infos, stem_infos)
            return
        offset = len(nodes)
        for t in transforms:
            nodes.append(_to_wp(t.position))
            scales.append(float(t.scale[0]))
        for i in range(len(transforms) - 1):
            edges.append((offset + i, offset + i + 1))
        if parent_node_idx is not None:
            edges.append((parent_node_idx, offset))
        if stem_infos is not None and blueprint.mesh_context is not None:
            stem_infos.append(_StemVisualInfo(
                blueprint=blueprint,
                node_indices=list(range(offset, offset + len(transforms))),
            ))
        for child in blueprint.nodes:
            _collect_graph(child.blueprint, nodes, edges, scales,
                           offset + child.parent_index,
                           include_stems, include_midrib, include_veins,
                           leaf_infos, stem_infos)

    elif isinstance(blueprint, LeafBlueprint):
        if not include_midrib:
            return
        transforms = blueprint.midrib.transforms
        if not transforms:
            return
        offset = len(nodes)
        midrib_indices = list(range(offset, offset + len(transforms)))
        for t in transforms:
            nodes.append(_to_wp(t.position))
            scales.append(float(t.scale[0]))
        for i in range(len(transforms) - 1):
            edges.append((offset + i, offset + i + 1))
        if parent_node_idx is not None:
            edges.append((parent_node_idx, offset))

        vein_indices: list[list[int]] = []
        if include_veins:
            for v_idx, vein in enumerate(blueprint.veins):
                if not vein.transforms:
                    vein_indices.append([])
                    continue
                midrib_node = offset + (v_idx // 2)
                vein_offset = len(nodes)
                vi = list(range(vein_offset, vein_offset + len(vein.transforms)))
                for t in vein.transforms:
                    nodes.append(_to_wp(t.position))
                    scales.append(float(t.scale[0]))
                edges.append((midrib_node, vein_offset))
                for j in range(len(vein.transforms) - 1):
                    edges.append((vein_offset + j, vein_offset + j + 1))
                vein_indices.append(vi)

        if leaf_infos is not None:
            leaf_infos.append(_LeafClothInfo(
                blueprint=blueprint,
                midrib_node_indices=midrib_indices,
                vein_node_indices=vein_indices,
            ))


def _deduplicate_nodes(
    nodes: list[wp.vec3],
    edges: list[tuple[int, int]],
    scales: list[float],
    min_dist: float = 1e-6,
) -> tuple[list[wp.vec3], list[tuple[int, int]], list[float], dict[int, int]]:
    """Merge nodes connected by zero-length edges via union-find. Drops self-loops."""
    n = len(nodes)
    parent = list(range(n))

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            if ra > rb:
                ra, rb = rb, ra
            parent[rb] = ra

    for u, v in edges:
        if float(wp.length(nodes[u] - nodes[v])) < min_dist:
            union(u, v)

    roots = [find(i) for i in range(n)]
    old_to_new: dict[int, int] = {}
    new_nodes: list[wp.vec3] = []
    new_scales: list[float] = []
    for i in range(n):
        r = roots[i]
        if r not in old_to_new:
            old_to_new[r] = len(new_nodes)
            new_nodes.append(nodes[r])
            new_scales.append(scales[r])
        old_to_new[i] = old_to_new[r]

    new_edges = [
        (old_to_new[u], old_to_new[v])
        for u, v in edges
        if old_to_new[u] != old_to_new[v]
    ]
    return new_nodes, new_edges, new_scales, old_to_new


def _add_plant_rods(
    builder: newton.ModelBuilder,
    nodes: list[wp.vec3],
    edges: list[tuple[int, int]],
    radii: list[float],
    bend_stiffness_modulus: float,
    bend_damping_modulus: float,
    stretch_stiffness_modulus: float,
    stretch_damping_modulus: float,
    label: str | None = None,
    fix_roots: bool = True,
) -> tuple[list[int], list[int], dict[int, int], dict[int, np.ndarray]]:
    """Add rod bodies and cable joints for a node/edge graph.

    Each edge becomes a capsule rigid body oriented along +Z from node u to v.
    Joints are built via BFS spanning forest and wrapped into articulations,
    following the same pattern as newton's ``add_rod_graph``.

    Returns (body_indices, joint_indices, node_to_body, body_xforms) where
    node_to_body maps each graph-node index to an incident body id, and
    body_xforms maps body id to its initial transform as [px,py,pz,qx,qy,qz,qw].
    """
    num_nodes = len(nodes)
    node_inc: list[list[int]] = [[] for _ in range(num_nodes)]
    edge_u: list[int] = []
    edge_v: list[int] = []
    edge_len: list[float] = []
    edge_bodies: list[int] = []
    body_xforms: dict[int, np.ndarray] = {}

    for u, v in edges:
        seg_vec = nodes[v] - nodes[u]
        seg_length = float(wp.length(seg_vec))
        if seg_length < 1e-9:
            continue

        q = quat_between_vectors_robust(wp.vec3(0.0, 0.0, 1.0), wp.normalize(seg_vec))
        half = 0.5 * seg_length
        idx = len(edge_bodies)

        r = (radii[u] + radii[v]) * 0.5

        xform = wp.transform(nodes[u], q)
        body_id = builder.add_link(
            xform=xform,
            com=wp.vec3(0.0, 0.0, half),
            label=f"{label}_b{idx}" if label else None,
        )
        body_xforms[body_id] = np.array([
            float(nodes[u][0]), float(nodes[u][1]), float(nodes[u][2]),
            float(q[0]), float(q[1]), float(q[2]), float(q[3]),
        ])
        builder.add_shape_capsule(
            body_id,
            xform=wp.transform(wp.vec3(0.0, 0.0, half), wp.quat_identity()),
            radius=r,
            half_height=half,
            cfg=newton.ModelBuilder.ShapeConfig(collision_group=-1)
        )

        edge_u.append(u)
        edge_v.append(v)
        edge_len.append(seg_length)
        edge_bodies.append(body_id)
        node_inc[u].append(idx)
        node_inc[v].append(idx)

    # Map each graph node to one of its incident bodies (used for cloth binding
    # and stem skinning — any incident body is fine since they meet at the node)
    node_to_body: dict[int, int] = {}
    for idx2 in range(len(edge_bodies)):
        node_to_body.setdefault(edge_u[idx2], edge_bodies[idx2])
        node_to_body.setdefault(edge_v[idx2], edge_bodies[idx2])

    if not edge_bodies:
        return [], [], {}, {}

    # BFS spanning forest: connect bodies with cable joints, then wrap
    # each connected component into an articulation for the solver
    all_joints: list[int] = []
    visited = [False] * len(edge_bodies)

    for start in range(len(edge_bodies)):
        if visited[start]:
            continue

        # Fix root body (make kinematic)
        if fix_roots:
            root = edge_bodies[start]
            builder.body_mass[root] = 0.0
            builder.body_inv_mass[root] = 0.0
            builder.body_inertia[root] = wp.mat33(0.0)
            builder.body_inv_inertia[root] = wp.mat33(0.0)
        
        queue = deque([start])
        visited[start] = True
        comp_joints: list[int] = []

        while queue:
            pe = queue.popleft()
            for nid in (edge_u[pe], edge_v[pe]):
                for ce in node_inc[nid]:
                    if ce == pe or visited[ce]:
                        continue
                    
                    pz = edge_len[pe] if nid == edge_v[pe] else 0.0
                    cz = edge_len[ce] if nid == edge_v[ce] else 0.0
                    L = 0.5 * (edge_len[pe] + edge_len[ce])
                    r = radii[nid]
                    r2 = r * r
                    r4 = r2 * r2
                    print(f"Connecting body {edge_bodies[pe]} to {edge_bodies[ce]} with length {L:.4f} and radius {r:.4f} r2={r2:.6f} r4={r4:.6f}")

                    j = builder.add_joint_cable(
                        parent=edge_bodies[pe],
                        child=edge_bodies[ce],
                        parent_xform=wp.transform(
                            wp.vec3(0.0, 0.0, pz), wp.quat_identity()
                        ),
                        child_xform=wp.transform(
                            wp.vec3(0.0, 0.0, cz), wp.quat_identity()
                        ),
                        bend_stiffness=bend_stiffness_modulus * r4 / L,
                        bend_damping=bend_damping_modulus * r4 / L,
                        stretch_stiffness=stretch_stiffness_modulus * r2 / L,
                        stretch_damping=stretch_damping_modulus * r2 / L,
                        collision_filter_parent=True,
                        enabled=True,
                    )
                    comp_joints.append(j)
                    all_joints.append(j)
                    visited[ce] = True
                    queue.append(ce)

        if comp_joints:
            builder.add_articulation(comp_joints, label=label)

    return edge_bodies, all_joints, node_to_body, body_xforms


def _build_leaf_cloth(
    builder: newton.ModelBuilder,
    leaf_info: _LeafClothInfo,
    node_to_body: dict[int, int],
    nodes: list[wp.vec3],
    cloth_density: float,
    tri_ke: float,
    tri_ka: float,
    tri_kd: float,
    edge_ke: float,
    edge_kd: float,
    particle_radius: float,
    bind_distance: float = 1e-4,
) -> tuple[ClothBinding, SkinBinding | None]:
    """Build a cloth mesh for a single leaf and bind particles to rod bodies.

    Also computes a barycentric mapping from the visual mesh to the collider
    mesh so a higher-resolution visual can be skinned at runtime.
    """
    bp = leaf_info.blueprint

    if bp.mesh_context is None:
        return ClothBinding(), None

    collider = bp.mesh_context.generate_collider(bp)
    if collider.triangle_indices is None or len(collider.vertex_positions) == 0:
        return ClothBinding(), None

    vertices = [_to_wp(v) for v in collider.vertex_positions]
    indices = collider.triangle_indices.flatten().tolist()

    if not indices:
        return ClothBinding(), None

    # ── Add cloth mesh (each vertex becomes a simulated particle) ──
    particle_start = len(builder.particle_q)
    builder.add_cloth_mesh(
        pos=wp.vec3(0.0, 0.0, 0.0),
        rot=wp.quat_identity(),
        scale=1.0,
        vel=wp.vec3(0.0, 0.0, 0.0),
        vertices=vertices,
        indices=indices,
        density=cloth_density,
        tri_ke=tri_ke,
        tri_ka=tri_ka,
        tri_kd=tri_kd,
        edge_ke=edge_ke,
        edge_kd=edge_kd,
        particle_radius=particle_radius,
    )

    # ── Pin cloth particles near rod skeleton nodes ──
    # Particles within bind_distance of a rod node are made kinematic:
    # they'll be driven by bind_particles_to_bodies each substep.
    binding = ClothBinding()
    node_positions = np.array(
        [[float(n[0]), float(n[1]), float(n[2])] for n in nodes]
    )
    for local_idx, vtx_pos in enumerate(collider.vertex_positions):
        dists = np.linalg.norm(node_positions - vtx_pos, axis=1)
        nearest_node = int(np.argmin(dists))
        if dists[nearest_node] > bind_distance:
            continue
        if nearest_node not in node_to_body:
            continue
        p_idx = particle_start + local_idx
        binding.bind_particle_ids.append(p_idx)
        binding.bind_body_ids.append(node_to_body[nearest_node])
        # Deactivate so the solver doesn't simulate this particle freely
        builder.particle_flags[p_idx] = (
            builder.particle_flags[p_idx] & ~ParticleFlags.ACTIVE
        )
        builder.particle_mass[p_idx] = 0.0

    # ── Map high-res visual mesh onto the low-res collider via barycentric coords ──
    visual = bp.mesh_context.generate_visual(bp)
    if visual.triangle_indices is None or len(visual.vertex_positions) == 0:
        return binding, None

    collider_tris = collider.triangle_indices  # (T, 3) local indices
    face_ids, bary_u, bary_v = _compute_barycentric_mapping(
        visual.vertex_positions, collider.vertex_positions, collider_tris,
    )
    # For leaves, control points = cloth particles (driven by VBD solver)
    num_cp = len(collider.vertex_positions)

    skin = SkinBinding(
        visual_indices=visual.triangle_indices.astype(np.int32),
        num_visual_verts=len(visual.vertex_positions),
        num_control_points=num_cp,
        cp_particle_ids=np.arange(particle_start, particle_start + num_cp, dtype=np.int32),
        cp_body_ids=np.full(num_cp, -1, dtype=np.int32),
        cp_local_offsets=np.zeros((num_cp, 3), dtype=np.float32),
        tri_indices=collider_tris.astype(np.int32),
        face_ids=face_ids,
        bary_u=bary_u,
        bary_v=bary_v,
    )
    return binding, skin


def _inverse_transform_point(body_origin: np.ndarray, world_pos: np.ndarray) -> np.ndarray:
    """Compute body-local position from world position and body transform.

    ``body_origin`` is [px, py, pz, qx, qy, qz, qw].
    Applies the inverse rotation (quaternion conjugate) to (world_pos - body_pos).
    """
    body_pos = body_origin[:3]
    bq = body_origin[3:7]  # (qx, qy, qz, qw)
    diff = world_pos - body_pos
    w = -bq[:3]  # conjugate imaginary part
    s = bq[3]    # real part
    t = 2.0 * np.cross(w, diff)
    return diff + s * t + np.cross(w, t)


def _build_stem_visual(
    stem_info: _StemVisualInfo,
    node_to_body: dict[int, int],
    body_xforms: dict[int, np.ndarray],
) -> SkinBinding | None:
    """Build a visual skin binding for a stem's visual mesh.

    Each collider vertex is bound to the nearest rod body as a rigid control
    point.  The visual mesh is then barycentric-mapped onto the collider
    triangles.
    """
    bp = stem_info.blueprint
    if bp.mesh_context is None:
        return None

    collider = bp.mesh_context.generate_collider(bp)
    if collider.triangle_indices is None or len(collider.vertex_positions) == 0:
        return None

    visual = bp.mesh_context.generate_visual(bp)
    if visual.triangle_indices is None or len(visual.vertex_positions) == 0:
        return None

    num_transforms = len(bp.transforms)
    num_cp = len(collider.vertex_positions)
    # The collider is an extruded tube: each ring of vertices corresponds
    # to one transform (cross-section frame) along the stem.
    vertex_per_ring = num_cp // num_transforms

    # All stem control points are body-sourced (no particles)
    cp_particle_ids = np.full(num_cp, -1, dtype=np.int32)
    cp_body_ids = np.empty(num_cp, dtype=np.int32)
    cp_local_offsets = np.empty((num_cp, 3), dtype=np.float32)

    for k in range(num_cp):
        # Map vertex → ring index → graph node → rod body
        frame_idx = min(k // vertex_per_ring, num_transforms - 1)
        graph_node = stem_info.node_indices[frame_idx]
        body_id = node_to_body.get(graph_node)
        if body_id is None:
            # Fallback: try adjacent frames
            for adj_offset in (1, -1, 2, -2):
                adj = frame_idx + adj_offset
                if 0 <= adj < num_transforms:
                    body_id = node_to_body.get(stem_info.node_indices[adj])
                    if body_id is not None:
                        break
        if body_id is None:
            cp_body_ids[k] = 0
            cp_local_offsets[k] = [0.0, 0.0, 0.0]
            continue

        cp_body_ids[k] = body_id
        # Store the vertex position in the body's local frame so
        # gather_skin_controls can reconstruct it from body_q later
        cp_local_offsets[k] = _inverse_transform_point(
            body_xforms[body_id], collider.vertex_positions[k],
        )

    collider_tris = collider.triangle_indices
    face_ids, bary_u, bary_v = _compute_barycentric_mapping(
        visual.vertex_positions, collider.vertex_positions, collider_tris,
    )

    return SkinBinding(
        visual_indices=visual.triangle_indices.astype(np.int32),
        num_visual_verts=len(visual.vertex_positions),
        num_control_points=num_cp,
        cp_particle_ids=cp_particle_ids,
        cp_body_ids=cp_body_ids,
        cp_local_offsets=cp_local_offsets.astype(np.float32),
        tri_indices=collider_tris.astype(np.int32),
        face_ids=face_ids,
        bary_u=bary_u,
        bary_v=bary_v,
    )


def compute_cloth_local_offsets(
    state: newton.State,
    bindings: ClothBinding,
    device=None,
) -> tuple[wp.array, wp.array, wp.array]:
    """Compute body-local offsets for bound particles after model finalization.

    Reads the finalized body and particle positions from state, then for each
    bound pair inverts the body transform to get the particle's position in
    body-local coordinates.  These offsets stay constant throughout simulation.

    Returns (bind_body_ids_wp, bind_particle_ids_wp, local_offsets_wp) ready
    for the ``bind_particles_to_bodies`` kernel.
    """
    body_q_np = state.body_q.numpy()
    particle_q_np = state.particle_q.numpy()
    offsets: list[wp.vec3] = []
    for body_idx, p_idx in zip(bindings.bind_body_ids, bindings.bind_particle_ids):
        tf = body_q_np[body_idx]
        body_pos = tf[:3]
        bq = tf[3:7]  # (qx, qy, qz, qw)
        p_pos = particle_q_np[p_idx]
        # Inverse quaternion rotation: conjugate(q) * (p - body_pos)
        diff = p_pos - body_pos
        w = np.array([-bq[0], -bq[1], -bq[2]])  # conjugate imaginary
        s = bq[3]
        t = 2.0 * np.cross(w, diff)
        local = diff + s * t + np.cross(w, t)
        offsets.append(wp.vec3(float(local[0]), float(local[1]), float(local[2])))

    return (
        wp.array(bindings.bind_body_ids, dtype=wp.int32, device=device),
        wp.array(bindings.bind_particle_ids, dtype=wp.int32, device=device),
        wp.array(offsets, dtype=wp.vec3, device=device),
    )


class ClothBindingHelper:
    """Manages GPU state for binding cloth particles to rigid bodies."""

    def __init__(self, state, cloth_bindings: ClothBinding, device=None):
        self.num_bindings = len(cloth_bindings.bind_particle_ids)
        if self.num_bindings > 0:
            self.bind_body_ids_wp, self.bind_particle_ids_wp, self.bind_local_offsets_wp = (
                compute_cloth_local_offsets(state, cloth_bindings, device=device)
            )

    def bind(self, state_0, state_1):
        """Update bound particle positions from body transforms."""
        if self.num_bindings == 0:
            return
        wp.launch(
            kernel=bind_particles_to_bodies,
            dim=self.num_bindings,
            inputs=[
                state_0.body_q,
                self.bind_body_ids_wp,
                self.bind_particle_ids_wp,
                self.bind_local_offsets_wp,
            ],
            outputs=[
                state_0.particle_q,
                state_1.particle_q,
            ],
        )


class PlantSkinRenderer:
    """Manages GPU skinning and rendering for visual plant meshes."""

    def __init__(self, skin: SkinBinding | None, device=None):
        self.active = skin is not None
        if not self.active:
            return
        assert skin is not None
        self.num_control_points = skin.num_control_points
        self.num_visual_verts = skin.num_visual_verts
        self.visual_indices_wp = wp.array(
            skin.visual_indices.flatten(), dtype=wp.int32, device=device,
        )
        self.cp_particle_ids_wp = wp.array(
            skin.cp_particle_ids, dtype=wp.int32, device=device,
        )
        self.cp_body_ids_wp = wp.array(
            skin.cp_body_ids, dtype=wp.int32, device=device,
        )
        self.cp_local_offsets_wp = wp.array(
            skin.cp_local_offsets, dtype=wp.vec3, device=device,
        )
        self.tri_indices_wp = wp.array(
            skin.tri_indices, dtype=wp.int32, ndim=2, device=device,
        )
        self.face_ids_wp = wp.array(skin.face_ids, dtype=wp.int32, device=device)
        self.bary_u_wp = wp.array(skin.bary_u, dtype=wp.float32, device=device)
        self.bary_v_wp = wp.array(skin.bary_v, dtype=wp.float32, device=device)
        self.control_positions_wp = wp.zeros(
            skin.num_control_points, dtype=wp.vec3, device=device,
        )
        self.visual_positions_wp = wp.zeros(
            skin.num_visual_verts, dtype=wp.vec3, device=device,
        )

    def update(self, state):
        """Launch GPU kernels to update visual mesh positions from state."""
        if not self.active:
            return
        wp.launch(
            kernel=gather_skin_controls,
            dim=self.num_control_points,
            inputs=[
                state.particle_q,
                state.body_q,
                self.cp_particle_ids_wp,
                self.cp_body_ids_wp,
                self.cp_local_offsets_wp,
            ],
            outputs=[self.control_positions_wp],
        )
        wp.launch(
            kernel=interpolate_skin_positions,
            dim=self.num_visual_verts,
            inputs=[
                self.control_positions_wp,
                self.tri_indices_wp,
                self.face_ids_wp,
                self.bary_u_wp,
                self.bary_v_wp,
            ],
            outputs=[self.visual_positions_wp],
        )

    def render(
        self,
        viewer,
        name: str = "/geo/visual_plant",
        color: tuple[float, float, float] = (0.2, 0.6, 0.1),
    ):
        """Render the skinned visual mesh to the viewer."""
        if not self.active:
            return
        viewer.log_mesh(
            name,
            self.visual_positions_wp,
            self.visual_indices_wp,
            backface_culling=False,
        )
        # Set color via the GL ObjectColor attribute.
        # HACK: MeshGL.update_texture() uploads the texture but never sets
        # the shader's texture_enable flag (attribute 8, w-component stays
        # 0.0), so the `texture` param of log_mesh is silently ignored.
        # TODO: Once Newton fixes MeshGL texture_enable, replace this with
        #       log_mesh(..., texture=<color image>) and drop the GL calls.
        mesh_gl = viewer.objects[name]
        from newton._src.viewer.gl.opengl import RendererGL
        gl = RendererGL.gl
        gl.glBindVertexArray(mesh_gl.vao)
        gl.glVertexAttrib3f(7, *color)
        gl.glBindVertexArray(0)


def build_newton_model(
    root: StemBlueprint,
    include_stems: bool = True,
    include_midrib: bool = True,
    include_veins: bool = True,
    include_cloth: bool = False,
    rod_radius: float = 0.02,
    bend_stiffness_modulus: float = 1.0e2,
    bend_damping_modulus: float = 1.0e-1,
    stretch_stiffness_modulus: float = 1.0e9,
    stretch_damping_modulus: float = 0.0,
    fix_root: bool = True,
    # Cloth parameters
    cloth_density: float = 1.0e-4,
    tri_ke: float = 1.0e4,
    tri_ka: float = 1.0e4,
    tri_kd: float = 1.0e-4,
    edge_ke: float = 1.0e0,
    edge_kd: float = 1.0e-2,
    particle_radius: float = 0.003,
) -> NewtonModelResult:
    """Build a Newton model with rod/cable structures from a plant blueprint tree.

    Toggle detail levels:
      include_stems  – add stem rods
      include_midrib – add leaf midrib rods (required for veins)
      include_veins  – add secondary vein rods branching from the midrib
      include_cloth  – add cloth mesh for each leaf, bound to rod bodies
    """
    builder = newton.ModelBuilder()

    nodes: list[wp.vec3] = []
    edges: list[tuple[int, int]] = []
    scales: list[float] = []
    leaf_infos: list[_LeafClothInfo] = []
    stem_infos: list[_StemVisualInfo] = []
    _collect_graph(
        root, nodes, edges, scales, None,
        include_stems, include_midrib, include_veins,
        leaf_infos if include_cloth else None,
        stem_infos,
    )
    # Merge overlapping nodes (e.g. where a stem tip meets a leaf base)
    nodes, edges, scales, old_to_new = _deduplicate_nodes(nodes, edges, scales)
    radii = [s * rod_radius for s in scales]

    # Remap node indices through deduplication
    for li in leaf_infos:
        li.midrib_node_indices = [old_to_new[i] for i in li.midrib_node_indices]
        li.vein_node_indices = [
            [old_to_new[j] for j in vi] for vi in li.vein_node_indices
        ]
    for si in stem_infos:
        si.node_indices = [old_to_new[i] for i in si.node_indices]

    node_to_body: dict[int, int] = {}
    body_xforms: dict[int, np.ndarray] = {}
    if edges:
        _, _, node_to_body, body_xforms = _add_plant_rods(
            builder,
            nodes,
            edges,
            radii,
            bend_stiffness_modulus,
            bend_damping_modulus,
            stretch_stiffness_modulus,
            stretch_damping_modulus,
            label="plant",
            fix_roots=fix_root,
        )

    # Build cloth meshes for leaves (simulated by VBD solver)
    cloth_bindings = ClothBinding()
    skin: SkinBinding | None = None
    if include_cloth:
        for li in leaf_infos:
            cb, leaf_skin = _build_leaf_cloth(
                builder, li, node_to_body, nodes,
                cloth_density, tri_ke, tri_ka, tri_kd,
                edge_ke, edge_kd, particle_radius,
            )
            cloth_bindings = cloth_bindings.merge(cb)
            if leaf_skin is not None:
                skin = leaf_skin if skin is None else skin.merge(leaf_skin)

    # Build visual skin for stems (bound rigidly to rod bodies)
    for si in stem_infos:
        stem_skin = _build_stem_visual(si, node_to_body, body_xforms)
        if stem_skin is not None:
            skin = stem_skin if skin is None else skin.merge(stem_skin)

    return NewtonModelResult(builder=builder, cloth_bindings=cloth_bindings, skin=skin)
