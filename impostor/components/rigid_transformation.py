from typing import Tuple
import numpy as np
from scipy.spatial.transform._rotation import Rotation
from dataclasses import dataclass, field

@dataclass
class RigidTransformation:
    rotation: Rotation = field(default_factory=lambda: Rotation.identity())
    translation: np.ndarray = field(default_factory=lambda: np.zeros(3))

    @classmethod
    def from_z_translation(cls, z: float) -> 'RigidTransformation':
        """
        Create a RigidTransformation that represents a translation along the z-axis.
        """
        return cls(translation=np.array([0, 0, z]))
    
    @classmethod
    def from_x_translation(cls, x: float) -> 'RigidTransformation':
        """
        Create a RigidTransformation that represents a translation along the x-axis.
        """
        return cls(translation=np.array([x, 0, 0]))
    

    def apply(self, point: np.ndarray) -> np.ndarray:
        """
        Apply the rigid transformation to a 3D point.
        
        Parameters:
        point (np.ndarray): A numpy array of shape (3,) representing the point in 3D space.
        
        Returns:
        np.ndarray: The transformed point as a numpy array of shape (3,).
        """
        assert point.shape == (3,), "Point must be a 3-dimensional vector."
        rotated_point = self.rotation.apply(point)
        transformed_point = rotated_point + self.translation
        return transformed_point

    def inverse(self) -> 'RigidTransformation':
        """
        Compute the inverse of the rigid transformation.
        
        Returns:
        RigidTransformation: A new RigidTransformation that is the inverse of the current one.
        """
        inverse_rotation = self.rotation.inv()
        inverse_translation = -inverse_rotation.apply(self.translation)
        return RigidTransformation(inverse_rotation, inverse_translation)

    def as_matrix(self) -> np.ndarray:
        """
        Get the transformation as a 4x4 homogeneous transformation matrix.
        
        Returns:
        np.ndarray: A 4x4 numpy array representing the homogeneous transformation matrix.
        """
        matrix = np.eye(4)
        matrix[:3, :3] = self.rotation.as_matrix()
        matrix[:3, 3] = self.translation
        return matrix

    @classmethod
    def from_matrix(cls, matrix: np.ndarray) -> 'RigidTransformation':
        """
        Create a RigidTransformation from a 4x4 homogeneous transformation matrix.
        
        Parameters:
        matrix (np.ndarray): A 4x4 numpy array representing the homogeneous transformation matrix.
        
        Returns:
        RigidTransformation: A new RigidTransformation object.
        """
        assert matrix.shape == (4, 4), "Matrix must be a 4x4 array."
        rotation = Rotation.from_matrix(matrix[:3, :3])
        translation = matrix[:3, 3]
        return cls(rotation, translation)

    def combine(self, other: 'RigidTransformation') -> 'RigidTransformation':
        """
        Combine this transformation with another one.
        
        Parameters:
        other (RigidTransformation): Another RigidTransformation object.
        
        Returns:
        RigidTransformation: A new RigidTransformation that is the combination of the two transformations.
        """
        combined_rotation = self.rotation * other.rotation
        combined_translation = self.rotation.apply(other.translation) + self.translation
        return RigidTransformation(combined_rotation, combined_translation)