
from pydantic import BaseModel, Field
import numpy as np
from scipy.spatial.transform import Rotation

class Transform(BaseModel):
    position: np.ndarray = Field(default_factory=lambda: np.zeros(3))
    rotation: Rotation = Field(default_factory=Rotation.identity)
    scale: np.ndarray = Field(default_factory=lambda: np.ones(3))

    @property
    def transform_matrix(self) -> np.ndarray:
        # Placeholder for actual transformation matrix calculation
        return np.eye(4)  # Identity matrix as a placeholder
    
    
class Material(BaseModel):
    color: np.ndarray = Field(default_factory=lambda: np.array([1.0, 1.0, 1.0]))
    texture: str | None = Field(default=None)

class Box(Transform, Material):
    size: np.ndarray = Field(default_factory=lambda: np.ones(3))    
