from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np
import newton
import warp as wp
from newton import ParticleFlags
from newton.math import quat_between_vectors_robust

from .mesh_builder import LeafBlueprint, StemBlueprint


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
class NewtonModelResult:
    """Result of build_newton_model: builder + cloth binding data."""
    builder: newton.ModelBuilder
    cloth_bindings: ClothBinding = field(default_factory=ClothBinding)


@dataclass
class _LeafClothInfo:
    """Internal: tracks graph-node indices for a single leaf."""
    blueprint: LeafBlueprint
    midrib_node_indices: list[int] = field(default_factory=list)
    vein_node_indices: list[list[int]] = field(default_factory=list)


@wp.kernel
def bind_particles_to_bodies(
    body_q: wp.array(dtype=wp.transform),
    bind_body_ids: wp.array(dtype=wp.int32),
    bind_particle_ids: wp.array(dtype=wp.int32),
    local_offsets: wp.array(dtype=wp.vec3),
    particle_q_0: wp.array(dtype=wp.vec3),
    particle_q_1: wp.array(dtype=wp.vec3),
):
    tid = wp.tid()
    body_idx = bind_body_ids[tid]
    p_idx = bind_particle_ids[tid]
    xform = body_q[body_idx]
    world_pos = wp.transform_point(xform, local_offsets[tid])
    particle_q_0[p_idx] = world_pos
    particle_q_1[p_idx] = world_pos


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
) -> None:
    """Recursively collect nodes and edges from the blueprint tree.

    Emits raw nodes/edges/scales without deduplication — call _deduplicate_nodes after.
    If *leaf_infos* is provided, appends a _LeafClothInfo for each leaf encountered.
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
                               leaf_infos)
            return
        offset = len(nodes)
        for t in transforms:
            nodes.append(_to_wp(t.position))
            scales.append(float(t.scale[0]))
        for i in range(len(transforms) - 1):
            edges.append((offset + i, offset + i + 1))
        if parent_node_idx is not None:
            edges.append((parent_node_idx, offset))
        for child in blueprint.nodes:
            _collect_graph(child.blueprint, nodes, edges, scales,
                           offset + child.parent_index,
                           include_stems, include_midrib, include_veins,
                           leaf_infos)

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
) -> tuple[list[int], list[int], dict[int, int]]:
    """Add rod bodies and cable joints for a node/edge graph.

    Each edge becomes a capsule rigid body oriented along +Z from node u to v.
    Joints are built via BFS spanning forest and wrapped into articulations,
    following the same pattern as newton's ``add_rod_graph``.

    Returns (body_indices, joint_indices, node_to_body) where node_to_body maps
    each graph-node index to an incident body id.
    """
    num_nodes = len(nodes)
    node_inc: list[list[int]] = [[] for _ in range(num_nodes)]
    edge_u: list[int] = []
    edge_v: list[int] = []
    edge_len: list[float] = []
    edge_bodies: list[int] = []

    for u, v in edges:
        seg_vec = nodes[v] - nodes[u]
        seg_length = float(wp.length(seg_vec))
        if seg_length < 1e-9:
            continue

        q = quat_between_vectors_robust(wp.vec3(0.0, 0.0, 1.0), wp.normalize(seg_vec))
        half = 0.5 * seg_length
        idx = len(edge_bodies)

        r = (radii[u] + radii[v]) * 0.5

        body_id = builder.add_link(
            xform=wp.transform(nodes[u], q),
            com=wp.vec3(0.0, 0.0, half),
            label=f"{label}_b{idx}" if label else None,
        )
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

    # Build node→body lookup (any incident body works for binding)
    node_to_body: dict[int, int] = {}
    for idx2 in range(len(edge_bodies)):
        node_to_body.setdefault(edge_u[idx2], edge_bodies[idx2])
        node_to_body.setdefault(edge_v[idx2], edge_bodies[idx2])

    if not edge_bodies:
        return [], [], {}


    # BFS spanning forest → cable joints → articulations
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

    return edge_bodies, all_joints, node_to_body


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
) -> ClothBinding:
    """Build a cloth mesh for a single leaf and bind particles to rod bodies."""
    bp = leaf_info.blueprint
    midrib_div = len(bp.midrib.transforms)
    if midrib_div < 2 or len(bp.veins) != (midrib_div - 1) * 2:
        return ClothBinding()

    # ── Build vertex layers (same algorithm as mesh_builder.generate_mesh) ──
    layers: list[list[wp.vec3]] = []
    # Parallel list of graph-node indices (or -1 for non-bound vertices)
    layer_node_ids: list[list[int]] = []

    midrib_ni = leaf_info.midrib_node_indices
    vein_ni = leaf_info.vein_node_indices

    for i in range(midrib_div):
        if i == midrib_div - 1:
            layers.append([nodes[midrib_ni[i]]])
            layer_node_ids.append([midrib_ni[i]])
        else:
            left_vein = bp.veins[i * 2]
            right_vein = bp.veins[i * 2 + 1]
            left_ni = vein_ni[i * 2] if i * 2 < len(vein_ni) else []
            right_ni = vein_ni[i * 2 + 1] if i * 2 + 1 < len(vein_ni) else []

            row_pos = (
                [_to_wp(t.position) for t in left_vein.transforms[::-1]]
                + [nodes[midrib_ni[i]]]
                + [_to_wp(t.position) for t in right_vein.transforms]
            )
            row_ids = (
                list(reversed(left_ni))
                + [midrib_ni[i]]
                + list(right_ni)
            )
            layers.append(row_pos)
            layer_node_ids.append(row_ids)

    # Flatten vertices
    vertices: list[wp.vec3] = []
    flat_node_ids: list[int] = []
    for row_pos, row_ids in zip(layers, layer_node_ids):
        vertices.extend(row_pos)
        flat_node_ids.extend(row_ids)

    # ── Triangulate between layers (progress-based, same as mesh_builder) ──
    indices: list[int] = []
    one_start = 0
    for i in range(len(layers) - 1):
        two_start = one_start + len(layers[i])
        one_id = 0
        two_id = 0
        one_size = len(layers[i])
        two_size = len(layers[i + 1])
        while one_id < one_size - 1 or two_id < two_size - 1:
            id1 = one_start + one_id % one_size
            id2 = two_start + two_id % two_size
            id1_next = one_start + (one_id + 1) % one_size
            id2_next = two_start + (two_id + 1) % two_size
            if one_id / one_size > two_id / two_size:
                indices.extend([id1, id2_next, id2])
                two_id += 1
            else:
                indices.extend([id1, id1_next, id2])
                one_id += 1
        one_start = two_start

    if not indices:
        return ClothBinding()

    # ── Add cloth mesh ──
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

    # ── Bind particles on midrib/vein positions to rod bodies ──
    binding = ClothBinding()
    for local_idx, graph_node in enumerate(flat_node_ids):
        if graph_node < 0 or graph_node not in node_to_body:
            continue
        p_idx = particle_start + local_idx
        binding.bind_particle_ids.append(p_idx)
        binding.bind_body_ids.append(node_to_body[graph_node])
        builder.particle_flags[p_idx] = (
            builder.particle_flags[p_idx] & ~ParticleFlags.ACTIVE
        )
        builder.particle_mass[p_idx] = 0.0

    return binding


def compute_cloth_local_offsets(
    state: newton.State,
    bindings: ClothBinding,
    device=None,
) -> tuple[wp.array, wp.array, wp.array]:
    """Compute body-local offsets for bound particles after model finalization.

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
        diff = p_pos - body_pos
        w = np.array([-bq[0], -bq[1], -bq[2]])
        s = bq[3]
        t = 2.0 * np.cross(w, diff)
        local = diff + s * t + np.cross(w, t)
        offsets.append(wp.vec3(float(local[0]), float(local[1]), float(local[2])))

    return (
        wp.array(bindings.bind_body_ids, dtype=wp.int32, device=device),
        wp.array(bindings.bind_particle_ids, dtype=wp.int32, device=device),
        wp.array(offsets, dtype=wp.vec3, device=device),
    )


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
    leaf_infos: list[_LeafClothInfo] = [] if include_cloth else [] # TODO why??
    _collect_graph(
        root, nodes, edges, scales, None,
        include_stems, include_midrib, include_veins,
        leaf_infos if include_cloth else None,
    )
    nodes, edges, scales, old_to_new = _deduplicate_nodes(nodes, edges, scales)
    radii = [s * rod_radius for s in scales]

    # Remap leaf_info node indices through deduplication
    for li in leaf_infos:
        li.midrib_node_indices = [old_to_new[i] for i in li.midrib_node_indices]
        li.vein_node_indices = [
            [old_to_new[j] for j in vi] for vi in li.vein_node_indices
        ]

    node_to_body: dict[int, int] = {}
    if edges:
        _, _, node_to_body = _add_plant_rods(
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

    # Build cloth meshes for leaves
    cloth_bindings = ClothBinding()
    if include_cloth:
        for li in leaf_infos:
            cb = _build_leaf_cloth(
                builder, li, node_to_body, nodes,
                cloth_density, tri_ke, tri_ka, tri_kd,
                edge_ke, edge_kd, particle_radius,
            )
            cloth_bindings = cloth_bindings.merge(cb)

    return NewtonModelResult(builder=builder, cloth_bindings=cloth_bindings)
