from dataclasses import dataclass
from typing import List, Tuple
from impostor.parts import BasePart, Entity
from impostor import parts
import torch
import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class Stick:
    a: int
    b: int


@dataclass
class Position:
    x: float
    y: float
    z: float


class ScaffoldingLayer(BasePart):
    # List of positions as [center, ...ring1, ...ring2]
    positions: torch.Tensor
    # Radius multiplyer of the ring1
    r1_scale: float
    # Radius multiplyer of the ring2
    r2_scale: float
    # Number of positions in the ring1
    n1: int
    # Number of positions in the ring2
    n2: int
    # Global indices for the positions in the layer
    indices: List[int]

    def __init__(
        self,
        global_index_start: int,
        r1_scale: float = 0.0,
        r2_scale: float = 0.0,
        n1: int = 0,
        n2: int = 0,
    ):
        assert n1 > 2, "we need at least 3 positions to create a layer"
        self.r1_scale = r1_scale
        self.r2_scale = r2_scale
        self.n1 = n1
        self.n2 = n2

        self.positions = torch.zeros(self.num_positions(), 3, dtype=torch.float32)
        # Save the global indices
        self.indices = list(
            range(global_index_start, global_index_start + self.num_positions())
        )

    def num_positions(self) -> int:
        return self.n1 + self.n2 + 1

    def update_positions(
        self, transform: parts.RigidTransformation, radius: float = 0.0
    ):
        """Update the positions in the layer based on a new transform"""
        center = torch.tensor(transform.translation, dtype=torch.float32)
        rotation: Rotation = transform.rotation

        def create_ring(n: int, r: float) -> torch.Tensor:
            ring = np.zeros((n, 3))
            for i in range(n):
                angle = 2 * np.pi * i / n
                ring[i] = [r * np.cos(angle), r * np.sin(angle), 0.0]
            # rotate the ring
            rotated_ring = rotation.apply(ring)
            # translate the ring to the center
            return torch.tensor(rotated_ring, dtype=torch.float32) + center

        self.positions = torch.zeros(self.num_positions(), 3, dtype=torch.float32)
        self.positions[0] = center
        if self.n1 > 0:
            self.positions[1 : self.n1 + 1] = create_ring(
                self.n1, radius * self.r1_scale
            )
        if self.n2 > 0:
            self.positions[self.n1 + 1 :] = create_ring(self.n2, radius * self.r2_scale)

    def get_positions(self) -> torch.Tensor:
        """Return all positions in the layer as (n, 3) tensor"""
        return self.positions

    def get_indices(self, entity: Entity) -> List[Tuple[Entity, int]]:
        """Return a list of indices for the positions in the layer"""
        return [(entity, i) for i in range(self.n1 + 1)]

    def global_idx(self, local_index: int) -> int:
        """Convert a local index to a global index"""
        return self.indices[local_index]
    
    def get_global_indices(self) -> List[int]:
        """Return the global indices of the positions in the layer"""
        return self.indices
    
    def get_global_center_index(self) -> int:
        """Return the global index of the center position"""
        return self.indices[0]

    def compute_bending(self, optimized_positions: torch.Tensor) -> Rotation:
        """Compute the rotation of the layer by finding the best fitting plane"""
        positions = optimized_positions[self.indices]

        try:
            # If positions are too close together, return identity rotation
            if (
                len(positions) < 3
                or torch.max(torch.norm(positions - positions[0], dim=1)) < 1e-6
            ):
                return Rotation.identity()

            # Compute the center of the ring
            center = torch.mean(positions, dim=0)

            # Compute the covariance matrix
            cov = torch.matmul((positions - center).T, positions - center)

            # Compute the SVD
            u, s, v = torch.svd(cov)

            # Ensure the resulting matrix is a proper rotation matrix (det = 1)
            rotation_matrix = torch.matmul(u, v.T)
            det = torch.det(rotation_matrix)

            # If determinant is negative, flip the last column to ensure det = 1
            if det < 0:
                u_adjusted = u.clone()
                u_adjusted[:, 2] = -u[:, 2]
                rotation_matrix = torch.matmul(u_adjusted, v.T)

            # Now create the rotation object
            return Rotation.from_matrix(rotation_matrix.numpy())
        except Exception as e:
            # Fallback to identity rotation if anything goes wrong
            print(
                f"Warning: Failed to compute rotation, falling back to identity. Error: {e}"
            )
            return Rotation.identity()

    def to_transform(
        self, optimized_positions: torch.Tensor
    ) -> parts.RigidTransformation:
        return parts.RigidTransformation(
            rotation=self.compute_bending(optimized_positions),
            translation=self.positions.numpy()[0],
        )
