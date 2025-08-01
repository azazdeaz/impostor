from pydantic import BaseModel, Field
import numpy as np
from scipy.spatial.transform import Rotation

class Transform3D(BaseModel):
    position: np.ndarray = Field(default_factory=lambda: np.zeros(3))
    rotation: Rotation = Field(default_factory=Rotation.identity)
    scale: np.ndarray = Field(default_factory=lambda: np.ones(3))

    class Config:
        arbitrary_types_allowed = True
    
    def as_matrix(self) -> np.ndarray:
        """Return the transform as a 4x4 transformation matrix."""
        matrix = np.identity(4)
        matrix[:3, :3] = self.rotation.as_matrix() * self.scale
        matrix[:3, 3] = self.position
        return matrix

    