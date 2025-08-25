
from pydantic import BaseModel, Field
import numpy as np
from scipy.spatial.transform import Rotation
from .transform_3d import Transform3D
import numpy.typing as npt

def quadratic_bezier(t: float, p0: np.ndarray, p1: np.ndarray, p2: np.ndarray) -> np.ndarray:
    """Calculates a point on a quadratic Bézier curve."""
    return (1 - t)**2 * p0 + 2 * (1 - t) * t * p1 + t**2 * p2

def quadratic_bezier_derivative(t: float, p0: np.ndarray, p1: np.ndarray, p2: np.ndarray) -> np.ndarray:
    """Calculates the derivative (tangent vector) of a quadratic Bézier curve."""
    return 2 * (1 - t) * (p1 - p0) + 2 * t * (p2 - p1)

class Trail(BaseModel):
    start_point: npt.NDArray[np.float64] = Field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))
    control_point: npt.NDArray[np.float64] = Field(default_factory=lambda: np.array([0.5, 0.0, 1.0]))
    end_point: npt.NDArray[np.float64] = Field(default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    steps: int = 10

    class Config:
        arbitrary_types_allowed = True

    def transforms(self) -> list[Transform3D]:
        """Generate a list of Transform3D objects representing the trail along a quadratic Bézier curve."""
        transforms: list[Transform3D] = []
        if self.steps <= 1:
            t = 0.0
            position = quadratic_bezier(t, self.start_point, self.control_point, self.end_point)
            tangent = quadratic_bezier_derivative(t, self.start_point, self.control_point, self.end_point)
            rotation = Rotation.align_vectors(np.array([[1, 0, 0]]), np.array([tangent]))[0]
            transforms.append(Transform3D(position=position, rotation=rotation, scale=np.ones(3)))
            return transforms

        for i in range(self.steps):
            t = i / (self.steps - 1)
            position = quadratic_bezier(t, self.start_point, self.control_point, self.end_point)
            
            # Calculate the tangent to find the direction
            tangent = quadratic_bezier_derivative(t, self.start_point, self.control_point, self.end_point)
            
            # Normalize the tangent vector
            z_axis = tangent / np.linalg.norm(tangent)
            
            # Define a constant "up" vector
            world_up = np.array([0.0, 1.0, 0.0])

            # Handle the case where the tangent is parallel to the world up vector
            if np.allclose(np.abs(np.dot(z_axis, world_up)), 1.0):
                # If parallel, use a different vector for the cross product
                x_axis = np.cross(np.array([0.0, 0.0, 1.0]), z_axis)
            else:
                x_axis = np.cross(world_up, z_axis)
            
            x_axis /= np.linalg.norm(x_axis)
            
            # Recalculate the up vector to be orthogonal
            y_axis = np.cross(z_axis, x_axis)
            
            # Create rotation from the basis vectors
            rotation_matrix = np.stack([x_axis, y_axis, z_axis], axis=-1)
            rotation = Rotation.from_matrix(rotation_matrix)
            
            transforms.append(Transform3D(position=position, rotation=rotation, scale=np.ones(3)))
            
        return transforms