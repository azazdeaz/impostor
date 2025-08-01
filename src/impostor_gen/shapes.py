
from pydantic import BaseModel, Field
import numpy as np
from .transform_3d import Transform3D
    
    
class Material(BaseModel):
    color: np.ndarray = Field(default_factory=lambda: np.array([1.0, 1.0, 1.0]))
    texture: str | None = Field(default=None)

class Box(Transform3D, Material):
    size: np.ndarray = Field(default_factory=lambda: np.ones(3))    
