from typing import List
import numpy as np
import trimesh

from impostor_gen.mesh2d import Mesh2D
from impostor_gen.mesh_utils import merge_meshes, transform_mesh, create_mesh_with_texture
from impostor_gen.transform_3d import Transform3D


def extrude_mesh2d_along_points(shape: Mesh2D, frames: list[Transform3D]) -> trimesh.Trimesh:
    """Extrude a 2D mesh along a series of 3D frames."""

    assert len(frames) > 0, "At least one frame is required for extrusion."

    shape3d = shape.as_mesh3d()
    transformed_meshes: List[trimesh.Trimesh] = []
    vertex_per_level = shape.vertex_positions.shape[0]

    for frame in frames:
        # Transform the 2D mesh to the current frame
        transformed_shape = transform_mesh(shape3d, frame)
        transformed_meshes.append(transformed_shape)

    # Merge all transformed shapes to get vertices
    merged_mesh = merge_meshes(transformed_meshes)
    
    # Build the triangle indices for the extruded mesh
    triangle_indices: List[List[int]] = []
    
    for i in range(len(frames) - 1):
        for [a, b] in shape.line_indices:
            v1 = i * vertex_per_level + a
            v2 = i * vertex_per_level + b
            v3 = (i + 1) * vertex_per_level + a
            v4 = (i + 1) * vertex_per_level + b
            triangle_indices.append([v1, v2, v3])
            triangle_indices.append([v2, v4, v3])
    
    # Build UVs
    vertex_texcoords = np.zeros((len(merged_mesh.vertices), 2), dtype=np.float32)
    uv_y_repeat = 2
    for i in range(len(frames)):
        v = i % uv_y_repeat / uv_y_repeat
        for j in range(vertex_per_level):
            u = j / (vertex_per_level - 1) if vertex_per_level > 1 else 0.0
            vertex_texcoords[i * vertex_per_level + j, :] = [u, v]
    
    # Create the final extruded mesh with proper faces and UVs
    return create_mesh_with_texture(
        vertices=merged_mesh.vertices,
        faces=np.array(triangle_indices, dtype=np.int32),
        vertex_texcoords=vertex_texcoords,
    )
