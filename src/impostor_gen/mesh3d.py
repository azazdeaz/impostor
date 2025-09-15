from typing import Optional
import numpy as np
from pydantic import BaseModel, Field, model_validator
from .transform_3d import Transform3D
import rerun as rr
import trimesh
import trimesh.visual.material
from PIL import Image


def concat_optional_arrays(
    arr1: Optional[np.ndarray],
    arr2: Optional[np.ndarray],
    offset: int = 0,
    padding_value: Optional[float] = None,
) -> Optional[np.ndarray]:
    """
    Concatenate two optional arrays.

    Args:
        arr1: The first array.
        arr2: The second array.
        offset: An integer value to add to the elements of the second array.
        padding_value: If one array is None, fill with this value.

    Returns:
        The concatenated array, or None if both are None.
    """
    if arr2 is not None and offset != 0:
        arr2 = arr2 + offset

    # Note: the weird conditional structuring is to calm the typechecker
    if arr1 is None:
        if arr2 is None:
            return None
        return (
            np.full((arr2.shape[0], arr2.shape[1]), padding_value, dtype=arr2.dtype)
            if padding_value is not None
            else arr2
        )

    if arr2 is None:
        return (
            np.full((arr1.shape[0], arr1.shape[1]), padding_value, dtype=arr1.dtype)
            if padding_value is not None
            else arr1
        )

    return (
        np.vstack((arr1, arr2))
        if padding_value is None
        else np.vstack(
            (
                arr1,
                np.full(
                    (arr2.shape[0], arr2.shape[1]), padding_value, dtype=arr2.dtype
                ),
            )
        )
    )


class Mesh3D(BaseModel):
    vertex_positions: np.ndarray = Field(
        ..., description="Array of vertex positions with shape (n, 3)"
    )
    vertex_normals: Optional[np.ndarray] = Field(
        default=None, description="Array of vertex normals with shape (n, 3)"
    )
    vertex_colors: Optional[np.ndarray] = Field(
        default=None, description="Array of vertex colors with shape (n, 3)"
    )
    triangle_indices: Optional[np.ndarray] = Field(
        default=None, description="Array of triangle indices with shape (m, 3)"
    )
    line_indices: Optional[np.ndarray] = Field(
        default=None, description="Array of line indices with shape (k, 2), optional"
    )
    vertex_texcoords: Optional[np.ndarray] = Field(
        default=None,
        description="Array of vertex texture (UV) coordinates with shape (n, 2)",
    )

    class Config:
        arbitrary_types_allowed = True

    @model_validator(mode="after")
    def validate_mesh(self) -> "Mesh3D":
        # Validate shapes
        if self.vertex_positions.ndim != 2 or self.vertex_positions.shape[1] != 3:
            raise ValueError("vertex_positions must be a 2D array with shape (n, 3)")
        if self.vertex_normals is not None:
            if self.vertex_normals.ndim != 2 or self.vertex_normals.shape[1] != 3:
                raise ValueError("vertex_normals must be a 2D array with shape (n, 3)")
        if self.vertex_colors is not None:
            if self.vertex_colors.ndim != 2 or self.vertex_colors.shape[1] != 3:
                raise ValueError("vertex_colors must be a 2D array with shape (n, 3)")
        if self.triangle_indices is not None:
            if self.triangle_indices.ndim != 2 or self.triangle_indices.shape[1] != 3:
                raise ValueError(
                    "triangle_indices must be a 2D array with shape (m, 3)"
                )
        if self.vertex_texcoords is not None:
            if self.vertex_texcoords.ndim != 2 or self.vertex_texcoords.shape[1] != 2:
                raise ValueError(
                    "vertex_texcoords must be a 2D array with shape (n, 2)"
                )

        # Validate triangle indices
        if self.triangle_indices is not None:
            if np.any(self.triangle_indices >= len(self.vertex_positions)):
                raise ValueError("triangle_indices must reference valid vertex indices")
            if np.any(self.triangle_indices < 0):
                raise ValueError(
                    "triangle_indices must reference non-negative vertex indices"
                )

        # Validate line indices if provided
        if self.line_indices is not None:
            if self.line_indices.ndim != 2 or self.line_indices.shape[1] != 2:
                raise ValueError("line_indices must be a 2D array with shape (k, 2)")
            if np.any(self.line_indices >= len(self.vertex_positions)):
                raise ValueError("line_indices must reference valid vertex indices")
            if np.any(self.line_indices < 0):
                raise ValueError(
                    "line_indices must reference non-negative vertex indices"
                )

        # Coerce types
        self.vertex_positions = self.vertex_positions.astype(np.float32)
        if self.vertex_normals is not None:
            self.vertex_normals = self.vertex_normals.astype(np.float32)
        if self.vertex_colors is not None:
            self.vertex_colors = self.vertex_colors.astype(np.float32)
        if self.triangle_indices is not None:
            self.triangle_indices = self.triangle_indices.astype(np.int32)
        if self.vertex_texcoords is not None:
            self.vertex_texcoords = self.vertex_texcoords.astype(np.float32)

        return self

    @classmethod
    def empty(cls) -> "Mesh3D":
        """Create an empty Mesh3D instance."""
        return cls(
            vertex_positions=np.empty((0, 3), dtype=np.float32),
        )

    def transform(self, transform: Transform3D) -> "Mesh3D":
        """Apply a 3D transformation to the mesh."""
        transformed_positions = (
            transform.position
            + self.vertex_positions @ transform.rotation.as_matrix().T * transform.scale
        )
        transformed_normals = None
        if self.vertex_normals is not None:
            transformed_normals = self.vertex_normals @ transform.rotation.as_matrix().T
        return Mesh3D(
            vertex_positions=transformed_positions,
            vertex_normals=transformed_normals,
            vertex_colors=self.vertex_colors,
            triangle_indices=self.triangle_indices,
            vertex_texcoords=self.vertex_texcoords,
        )

    def merge(self, other: "Mesh3D") -> "Mesh3D":
        """Merge this mesh with another Mesh3D instance."""
        offset = len(self.vertex_positions)

        return Mesh3D(
            vertex_positions=np.vstack((self.vertex_positions, other.vertex_positions)),
            vertex_normals=concat_optional_arrays(
                self.vertex_normals, other.vertex_normals
            ),
            vertex_colors=concat_optional_arrays(
                self.vertex_colors, other.vertex_colors
            ),
            triangle_indices=concat_optional_arrays(
                self.triangle_indices, other.triangle_indices, offset=offset
            ),
            line_indices=concat_optional_arrays(
                self.line_indices, other.line_indices, offset=offset
            ),
            vertex_texcoords=concat_optional_arrays(
                self.vertex_texcoords, other.vertex_texcoords
            ),
        )
    
    def to_rerun(self) -> rr.Mesh3D:
        """Convert this Mesh3D instance to a rerun.Mesh3D instance."""
        return rr.Mesh3D(
            vertex_positions=self.vertex_positions,
            vertex_normals=self.vertex_normals,
            vertex_colors=self.vertex_colors,
            triangle_indices=self.triangle_indices,
            vertex_texcoords=self.vertex_texcoords,
        )
    
    def to_trimesh(self) -> trimesh.Trimesh:
        """Convert this Mesh3D instance to a trimesh.Trimesh instance."""
        mesh =  trimesh.Trimesh(
            vertices=self.vertex_positions,
            faces=self.triangle_indices,
            vertex_normals=self.vertex_normals,
            vertex_colors=self.vertex_colors,
            process=False  # Disable automatic processing to preserve original data
        )

        if self.vertex_texcoords is not None:
            print(f"UVs: {self.vertex_texcoords}")
            material = trimesh.visual.material.PBRMaterial(
                baseColorTexture=Image.open("uv1.png"),
            )

            visual = trimesh.visual.TextureVisuals(
                uv=self.vertex_texcoords,
                material=material
            )
            mesh.visual = visual

        return mesh