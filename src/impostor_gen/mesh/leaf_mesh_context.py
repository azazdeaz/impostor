from __future__ import annotations

from typing import Any, List

import numpy as np

from .mesh_context import MeshContext
from .mesh3d import Mesh3D


def _triangulate_leaf(blueprint: Any) -> Mesh3D:
    """Build a triangulated Mesh3D from a LeafBlueprint's midrib + veins."""
    midrib_div = len(blueprint.midrib.transforms)

    layers: List[List[np.ndarray]] = []

    assert len(blueprint.veins) == (midrib_div - 1) * 2, (
        f"Each midrib section must have two secondary veins except the tip. "
        f"Found {len(blueprint.veins)} veins for {midrib_div} midrib sections."
    )

    for i in range(midrib_div):
        if i == midrib_div - 1:
            layers.append([blueprint.midrib.transforms[i].position])
        else:
            layers.append(
                [t.position for t in blueprint.veins[i * 2].transforms[::-1]]
                + [blueprint.midrib.transforms[i].position]
                + [t.position for t in blueprint.veins[i * 2 + 1].transforms]
            )

    vertices = np.concatenate(layers)
    one_start = 0
    faces: List[List[int]] = []
    is_closed = False
    winding_clockwise = True

    for i in range(len(layers) - 1):
        two_start = one_start + len(layers[i])
        one_id = 0
        two_id = 0
        one_size = len(layers[i])
        two_size = len(layers[i + 1])

        id_margin = 0 if is_closed else 1
        while one_id < one_size - id_margin or two_id < two_size - id_margin:
            id1 = one_start + one_id % one_size
            id2 = two_start + two_id % two_size
            id1_next = one_start + (one_id + 1) % one_size
            id2_next = two_start + (two_id + 1) % two_size

            up_progress = one_id / one_size
            down_progress = two_id / two_size

            if up_progress > down_progress:
                if winding_clockwise:
                    new_face = [id1, id2_next, id2]
                else:
                    new_face = [id1, id2, id2_next]
                two_id += 1
            else:
                if winding_clockwise:
                    new_face = [id1, id1_next, id2]
                else:
                    new_face = [id1, id2, id1_next]
                one_id += 1

            faces.append(new_face)

        one_start = two_start

    return Mesh3D(
        vertex_positions=vertices,
        triangle_indices=np.array(faces),
        material_key=blueprint.material_key,
    )


class LeafMeshContext(MeshContext):
    """Generates triangulated leaf meshes from midrib + vein blueprints."""

    def generate_visual(self, blueprint: Any) -> Mesh3D:
        return _triangulate_leaf(blueprint)

    def generate_collider(self, blueprint: Any) -> Mesh3D:
        return _triangulate_leaf(blueprint)
