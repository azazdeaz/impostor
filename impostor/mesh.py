from dataclasses import dataclass, field
from typing import List
import numpy as np


@dataclass
class Bone:
    body_name: str
    bind_position: np.ndarray  # 3D position
    bind_quaternion: np.ndarray  # 4D quaternion [w, x, y, z]
    vertex_indices: List[int]
    vertex_weights: List[float]
    radius: float = 0.001


@dataclass
class Mesh:
    vertices: np.ndarray = field(default_factory=lambda: np.ndarray((0, 3)))
    faces: np.ndarray = field(default_factory=lambda: np.ndarray((0, 3), dtype=int))
    normals: np.ndarray = field(default_factory=lambda: np.ndarray((0, 3)))
    uvs: np.ndarray = field(default_factory=lambda: np.ndarray((0, 2)))
    bones: List[Bone] = field(default_factory=list)

def compute_vertex_weights(
    mesh: Mesh, max_bones_per_vertex: int = 4, smoothness: float = 0.1
):
    """
    Compute vertex weights for all vertices in the mesh.
    For each vertex, find the closest bones and assign weights inversely proportional to distance.

    Args:
        mesh: The mesh to compute weights for
        max_bones_per_vertex: Maximum number of bones to influence each vertex
        distance_threshold: If distance to a bone is less than this, assign weight 1.0 to that bone only
    """
    if not mesh.bones:
        return  # No bones to compute weights for

    # Clear existing vertex assignments in bones
    for bone in mesh.bones:
        bone.vertex_indices = []
        bone.vertex_weights = []

    # For each vertex, compute weights for the closest bones
    for vertex_idx, vertex_position in enumerate(mesh.vertices):
        # Compute distances to all bones
        distances = []
        for bone_idx, bone in enumerate(mesh.bones):
            distance = np.linalg.norm(vertex_position - bone.bind_position)
            distances.append((bone_idx, distance))

        # Sort by distance
        distances.sort(key=lambda x: x[1])

        closest_bones_data = distances[:max_bones_per_vertex]
        max_distance = closest_bones_data[-1][1] if closest_bones_data else 0

        # Use a softmax-like weighting for smoothness
        weights = [((max_distance-dist) / max_distance) ** 2 for _, dist in closest_bones_data]
        total_weight = sum(weights)
        normalized_weights = [w / total_weight for w in weights]

        for i, (bone_idx, _) in enumerate(closest_bones_data):
            mesh.bones[bone_idx].vertex_indices.append(vertex_idx)
            mesh.bones[bone_idx].vertex_weights.append(normalized_weights[i])
