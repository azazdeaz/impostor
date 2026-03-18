from collections import deque

import newton
import warp as wp

from newton.math import quat_between_vectors_robust

from .mesh_builder import LeafBlueprint, StemBlueprint, Turtle


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
) -> None:
    """Recursively collect nodes and edges from the blueprint tree.

    Emits raw nodes/edges/scales without deduplication — call _deduplicate_nodes after.
    """
    if isinstance(blueprint, StemBlueprint):
        if not include_stems:
            return
        transforms = blueprint.transforms
        if not transforms:
            for child in blueprint.nodes:
                _collect_graph(child.blueprint, nodes, edges, scales,
                               parent_node_idx,
                               include_stems, include_midrib, include_veins)
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
                           offset + child.transform_index,
                           include_stems, include_midrib, include_veins)

    elif isinstance(blueprint, LeafBlueprint):
        if not include_midrib:
            return
        transforms = blueprint.midrib.transforms
        if not transforms:
            return
        offset = len(nodes)
        for t in transforms:
            nodes.append(_to_wp(t.position))
            scales.append(float(t.scale[0]))
        for i in range(len(transforms) - 1):
            edges.append((offset + i, offset + i + 1))
        if parent_node_idx is not None:
            edges.append((parent_node_idx, offset))
        if include_veins:
            for v_idx, vein in enumerate(blueprint.veins):
                if not vein.transforms:
                    continue
                midrib_node = offset + (v_idx // 2)
                vein_offset = len(nodes)
                for t in vein.transforms:
                    nodes.append(_to_wp(t.position))
                    scales.append(float(t.scale[0]))
                edges.append((midrib_node, vein_offset))
                for j in range(len(vein.transforms) - 1):
                    edges.append((vein_offset + j, vein_offset + j + 1))


def _deduplicate_nodes(
    nodes: list[wp.vec3],
    edges: list[tuple[int, int]],
    scales: list[float],
    min_dist: float = 1e-6,
) -> tuple[list[wp.vec3], list[tuple[int, int]], list[float]]:
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
    return new_nodes, new_edges, new_scales


def _add_plant_rods(
    builder: newton.ModelBuilder,
    nodes: list[wp.vec3],
    edges: list[tuple[int, int]],
    radii: list[float],
    bend_stiffness: float,
    bend_damping: float,
    stretch_stiffness: float,
    stretch_damping: float,
    label: str | None = None,
    fix_roots: bool = True,
) -> tuple[list[int], list[int]]:
    """Add rod bodies and cable joints for a node/edge graph.

    Each edge becomes a capsule rigid body oriented along +Z from node u to v.
    Joints are built via BFS spanning forest and wrapped into articulations,
    following the same pattern as newton's ``add_rod_graph``.

    Returns (body_indices, joint_indices).
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

    if not edge_bodies:
        return [], []


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

                    j = builder.add_joint_cable(
                        parent=edge_bodies[pe],
                        child=edge_bodies[ce],
                        parent_xform=wp.transform(
                            wp.vec3(0.0, 0.0, pz), wp.quat_identity()
                        ),
                        child_xform=wp.transform(
                            wp.vec3(0.0, 0.0, cz), wp.quat_identity()
                        ),
                        bend_stiffness=bend_stiffness / L,
                        bend_damping=bend_damping,
                        stretch_stiffness=stretch_stiffness / L,
                        stretch_damping=stretch_damping,
                        collision_filter_parent=True,
                        enabled=True,
                    )
                    comp_joints.append(j)
                    all_joints.append(j)
                    visited[ce] = True
                    queue.append(ce)

        if comp_joints:
            builder.add_articulation(comp_joints, label=label)

    return edge_bodies, all_joints


def build_newton_model(
    root: StemBlueprint,
    include_stems: bool = True,
    include_midrib: bool = True,
    include_veins: bool = True,
    rod_radius: float = 0.02,
    bend_stiffness: float = 1.0e2,
    bend_damping: float = 1.0e-1,
    stretch_stiffness: float = 1.0e9,
    stretch_damping: float = 0.0,
    fix_root: bool = True,
) -> newton.ModelBuilder:
    """Build a Newton model with rod/cable structures from a plant blueprint tree.

    Toggle detail levels:
      include_stems  – add stem rods
      include_midrib – add leaf midrib rods (required for veins)
      include_veins  – add secondary vein rods branching from the midrib
    """
    builder = newton.ModelBuilder()

    nodes: list[wp.vec3] = []
    edges: list[tuple[int, int]] = []
    scales: list[float] = []
    _collect_graph(
        root, nodes, edges, scales, None,
        include_stems, include_midrib, include_veins,
    )
    nodes, edges, scales = _deduplicate_nodes(nodes, edges, scales)
    radii = [s * rod_radius for s in scales]

    if edges:
        _add_plant_rods(
            builder,
            nodes,
            edges,
            radii,
            bend_stiffness,
            bend_damping,
            stretch_stiffness,
            stretch_damping,
            label="plant",
            fix_roots=fix_root,
        )

    return builder
