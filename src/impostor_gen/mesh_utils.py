from typing import List, Optional
import numpy as np
import trimesh
import trimesh.util
import trimesh.visual
import trimesh.visual.material
import rerun as rr
from PIL import Image

from .transform_3d import Transform3D


def merge_meshes(meshes: List[trimesh.Trimesh]) -> trimesh.Trimesh:
    """Merge multiple trimesh.Trimesh instances using trimesh.util.concatenate."""
    if not meshes:
        return empty_mesh()
    
    # Filter out empty meshes
    non_empty_meshes = [mesh for mesh in meshes if len(mesh.vertices) > 0]
    
    if not non_empty_meshes:
        return empty_mesh()
    
    if len(non_empty_meshes) == 1:
        return non_empty_meshes[0].copy()
    
    return trimesh.util.concatenate(non_empty_meshes) # type: ignore

def empty_mesh() -> trimesh.Trimesh:
    """Create an empty trimesh.Trimesh instance."""
    return trimesh.Trimesh(vertices=np.empty((0, 3)), faces=np.empty((0, 3), dtype=np.int32))

def transform_mesh(mesh: trimesh.Trimesh, transform: Transform3D) -> trimesh.Trimesh:
    """Apply a 3D transformation to a trimesh.Trimesh."""
    if len(mesh.vertices) == 0:
        return mesh.copy()
    
    # Create transformation matrix
    # trimesh expects 4x4 transformation matrices
    transform_matrix = np.eye(4)
    
    # Apply rotation and scale
    rotation_matrix = transform.rotation.as_matrix() * transform.scale
    transform_matrix[:3, :3] = rotation_matrix
    
    # Apply translation
    transform_matrix[:3, 3] = transform.position
    
    # Apply transformation
    transformed_mesh = mesh.copy()
    transformed_mesh.apply_transform(transform_matrix)
    
    return transformed_mesh


def log_mesh(mesh: trimesh.Trimesh | List[trimesh.Trimesh]):
    """Convert a trimesh.Trimesh instance to a rerun.Asset3D."""
    if not isinstance(mesh, list):
        mesh = [mesh]
    
    for i, m in enumerate(mesh):
        asset: rr.datatypes.Blob = m.export(file_type='glb')  # type: ignore
        rr.log(f"mesh/part_{i}", rr.Asset3D(contents=asset))


def create_mesh_with_texture(
    vertices: np.ndarray,
    faces: np.ndarray,
    vertex_texcoords: Optional[np.ndarray] = None,
    vertex_normals: Optional[np.ndarray] = None,
    vertex_colors: Optional[np.ndarray] = None,
    texture_path: Optional[str] = "uv1.png"
) -> trimesh.Trimesh:
    """Create a trimesh.Trimesh with optional texture coordinates and material."""
    mesh = trimesh.Trimesh(
        vertices=vertices,
        faces=faces,
        vertex_normals=vertex_normals,
        vertex_colors=vertex_colors,
        process=False  # Disable automatic processing to preserve original data
    )

    if vertex_texcoords is not None and texture_path is not None:
        material = trimesh.visual.material.PBRMaterial(
            baseColorTexture=Image.open(texture_path),
        )

        visual = trimesh.visual.TextureVisuals(
            uv=vertex_texcoords,
            material=material
        )
        mesh.visual = visual

    return mesh