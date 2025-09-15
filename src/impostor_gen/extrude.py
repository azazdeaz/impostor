from typing import List
import numpy as np

from impostor_gen.mesh2d import Mesh2D
from impostor_gen.mesh3d import Mesh3D
from impostor_gen.transform_3d import Transform3D


def extrude_mesh2d_along_points(shape: Mesh2D, frames: list[Transform3D]) -> Mesh3D:
    """Extrude a 2D mesh along a series of 3D frames."""

    assert len(frames) > 0, "At least one frame is required for extrusion."

    shape3d = shape.as_mesh3d()
    extruded_mesh = Mesh3D.empty()
    vertex_per_level = shape.vertex_positions.shape[0]

    for frame in frames:
        # Transform the 2D mesh to the current frame
        transformed_shape = shape3d.transform(frame)
        extruded_mesh = extruded_mesh.merge(transformed_shape)

    triangle_indices: List[List[int]] = []
    extruded_mesh.vertex_texcoords = np.zeros((len(extruded_mesh.vertex_positions), 2), dtype=np.float32)

    # Build the triangle indices for the extruded mesh
    for i in range(len(frames) - 1):
        for [a, b] in shape.line_indices:
            v1 = i * vertex_per_level + a
            v2 = i * vertex_per_level + b
            v3 = (i + 1) * vertex_per_level + a
            v4 = (i + 1) * vertex_per_level + b
            triangle_indices.append([v1, v2, v3])
            triangle_indices.append([v2, v4, v3])
    
    # Build UVs
    uv_y_repeat = 2
    for i in range(len(frames)):
        v = i % uv_y_repeat / uv_y_repeat
        for j in range(vertex_per_level):
            u = j / (vertex_per_level - 1) if vertex_per_level > 1 else 0.0
            extruded_mesh.vertex_texcoords[i * vertex_per_level + j, :] = [u, v]
    
    extruded_mesh.triangle_indices = np.array(triangle_indices, dtype=np.int32)
    
    extruded_mesh.line_indices = None
    return extruded_mesh
