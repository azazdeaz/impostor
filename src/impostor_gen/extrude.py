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

    # Build the triangle indices for the extruded mesh
    for i in range(len(frames) - 1):
        for [a, b] in shape.line_indices:
            v1 = i * vertex_per_level + a
            v2 = i * vertex_per_level + b
            v3 = (i + 1) * vertex_per_level + a
            v4 = (i + 1) * vertex_per_level + b
            new_triangle_indices = np.array([[v1, v2, v3], [v2, v4, v3]])
            extruded_mesh.triangle_indices = (
                np.vstack((extruded_mesh.triangle_indices, new_triangle_indices))
                if extruded_mesh.triangle_indices is not None
                else new_triangle_indices
            )
    
    extruded_mesh.line_indices = None
    return extruded_mesh
