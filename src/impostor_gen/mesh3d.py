import numpy as np
from pydantic import BaseModel, Field

class Mesh3D(BaseModel):
    vertices: np.ndarray = Field(..., description="Array of vertex positions with shape (n, 3)")
    faces: np.ndarray = Field(..., description="Array of face indices with shape (m, 3)")