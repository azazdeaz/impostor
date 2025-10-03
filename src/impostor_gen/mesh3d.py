import random
import string
from typing import List, Optional

import numpy as np
import rerun as rr
import trimesh
from pxr import Kind, Sdf, Usd, UsdGeom, UsdShade
from pydantic import BaseModel, Field, model_validator

from impostor_gen.material import MaterialRegistry

from .transform_3d import Transform3D


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
    name: str = Field(default="mesh", description="Unique name for the mesh")
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
    material_key: Optional[str] = Field(
        default=None, description="Key to look up the material in a material dictionary"
    )

    class Config:
        arbitrary_types_allowed = True

    @model_validator(mode="after")
    def validate_mesh(self) -> "Mesh3D":
        # Make sure name is unique

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

    def to_trimesh(self, materials: MaterialRegistry) -> trimesh.Trimesh:
        """Convert this Mesh3D instance to a trimesh.Trimesh instance."""
        mesh = trimesh.Trimesh(
            vertices=self.vertex_positions,
            faces=self.triangle_indices,
            vertex_normals=self.vertex_normals,
            vertex_colors=self.vertex_colors,
            process=False,  # Disable automatic processing to preserve original data
        )

        material = materials.get(self.material_key) if self.material_key else None

        if self.vertex_texcoords is not None and material is not None:
            tm_material = material.to_trimesh() if material else None

            visual = trimesh.visual.TextureVisuals(  # type: ignore
                uv=self.vertex_texcoords, material=tm_material
            )
            mesh.visual = visual

        return mesh

    def to_usd(
        self,
        stage: Optional[Usd.Stage],
        materials: MaterialRegistry,
        parent: Optional[str] = None,
    ) -> Usd.Stage:
        if stage is None:
            stage = Usd.Stage.CreateNew("simpleShading.usda")

        name = (
            self.name
            + "_"
            + "".join(random.choices(string.ascii_lowercase + string.digits, k=4))
        )
        if parent is not None:
            name = f"{parent}/{name}"
        if not name.startswith("/"):
            name = f"/{name}"

        material = materials.get(self.material_key) if self.material_key else None

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)

        modelRoot = UsdGeom.Xform.Define(stage, f"{name}")
        Usd.ModelAPI(modelRoot).SetKind(Kind.Tokens.component)

        mesh = UsdGeom.Mesh.Define(stage, f"{name}/mesh")
        mesh.CreatePointsAttr(self.vertex_positions.tolist())
        if self.triangle_indices is not None:
            face_vertex_counts = [3] * len(self.triangle_indices)
            face_vertex_indices = self.triangle_indices.flatten().tolist()
            mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
            mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)
        if self.vertex_texcoords is not None and material is not None:
            texCoords = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.varying
            )
            texCoords.Set(self.vertex_texcoords.tolist())

            usd_material = material.to_usd(stage, f"{name}/material")

            mesh.GetPrim().ApplyAPI(UsdShade.MaterialBindingAPI)  # type: ignore
            UsdShade.MaterialBindingAPI(mesh).Bind(usd_material)

        return stage


class CompundMesh3D(BaseModel):
    submeshes: List[Mesh3D] = Field(default_factory=lambda: [])

    def merge(self, other: "CompundMesh3D | Mesh3D") -> "CompundMesh3D":
        if isinstance(other, Mesh3D):
            return CompundMesh3D(submeshes=self.submeshes + [other])
        else:
            return CompundMesh3D(submeshes=self.submeshes + other.submeshes)

    def to_usd(
        self, materials: MaterialRegistry, filename: str = "compound_mesh.usda"
    ) -> "Usd.Stage":
        stage = Usd.Stage.CreateNew(filename)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

        for mesh in self.submeshes:
            mesh.to_usd(stage, materials)

        return stage
