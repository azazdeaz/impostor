from typing import Annotated

import numpy as np
import rerun as rr
from numpy.typing import NDArray
from pydantic import BaseModel, Field
from scipy.spatial.transform import Rotation

# Specify both dtype and shape
Vector3 = Annotated[NDArray[np.float64], (3,)]


class Transform3D(BaseModel):
    position: Vector3 = Field(default_factory=lambda: np.zeros(3))
    rotation: Rotation = Field(default_factory=Rotation.identity)
    scale: Vector3 = Field(default_factory=lambda: np.ones(3))

    class Config:
        arbitrary_types_allowed = True

    def set_scale(self, scale: float):
        self.scale = np.array([scale, scale, scale])

    def as_matrix(self) -> np.ndarray:
        """Return the transform as a 4x4 transformation matrix."""
        matrix = np.identity(4)
        matrix[:3, :3] = self.rotation.as_matrix() * self.scale
        matrix[:3, 3] = self.position
        return matrix

    def to_rerun(self) -> rr.Transform3D:
        """Convert the Transform3D to a Rerun Transform3D."""
        return rr.Transform3D(
            translation=self.position,
            rotation=rr.Quaternion(xyzw=self.rotation.as_quat()),
            scale=self.scale,
        )
