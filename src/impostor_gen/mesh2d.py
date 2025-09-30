from typing import Optional
import numpy as np
from pydantic import BaseModel, Field, model_validator
from impostor_gen.mesh3d import Mesh3D


class Mesh2D(BaseModel):
    vertex_positions: np.ndarray = Field(
        ..., description="Array of vertex positions with shape (n, 2)"
    )
    line_indices: np.ndarray = Field(
        ..., description="Array of edge indices with shape (m, 2)"
    )
    vertex_normals: Optional[np.ndarray] = Field(
        default=None, description="Array of vertex normals with shape (n, 2), optional"
    )

    class Config:
        arbitrary_types_allowed = True

    @model_validator(mode="after")
    def validate_mesh(self) -> "Mesh2D":
        # Validate vertices
        if self.vertex_positions.ndim != 2 or self.vertex_positions.shape[1] != 2:
            raise ValueError("Vertices must be a 2D array with shape (n, 2)")

        # Validate edges
        if self.line_indices.ndim != 2 or self.line_indices.shape[1] != 2:
            raise ValueError("Edges must be a 2D array with shape (m, 2)")

        # Validate edge indices
        if np.any(self.line_indices >= len(self.vertex_positions)):
            raise ValueError("Edges must reference valid vertex indices")
        if np.any(self.line_indices < 0):
            raise ValueError("Edges must reference non-negative vertex indices")

        # Coerce types
        self.vertex_positions = self.vertex_positions.astype(np.float32)
        self.line_indices = self.line_indices.astype(np.int32)

        return self

    def __repr__(self):
        return f"Mesh2D(vertices={self.vertex_positions.shape[0]}, edges={self.line_indices.shape[0]})"

    def __len__(self):
        return len(self.line_indices)

    def as_mesh3d(self, z_height: float = 0.0) -> Mesh3D:
        """Convert this 2D mesh to a 3D mesh by extruding along the z-axis."""
        vertex_positions_3d = np.hstack(
            (
                self.vertex_positions,
                np.full((self.vertex_positions.shape[0], 1), z_height),
            )
        )
        if self.vertex_normals is not None:
            vertex_normals_3d = np.hstack(
                (self.vertex_normals, np.zeros((self.vertex_normals.shape[0], 1)))
            )
        else:
            vertex_normals_3d = None

        return Mesh3D(
            vertex_positions=vertex_positions_3d,
            line_indices=self.line_indices,
            vertex_normals=vertex_normals_3d,
        )

    @classmethod
    def circle(cls, radius: float = 1.0, segments: int = 16) -> "Mesh2D":
        """Create a circular 2D mesh."""
        angles = np.linspace(0, 2 * np.pi, segments, endpoint=False)
        vertices = np.stack((radius * np.cos(angles), radius * np.sin(angles)), axis=-1)
        normals = vertices / np.linalg.norm(vertices, axis=1, keepdims=True)
        edges = np.array(
            [[i, (i + 1) % segments] for i in range(segments)], dtype=np.int32
        )
        return Mesh2D(
            vertex_positions=vertices, line_indices=edges, vertex_normals=normals
        )
