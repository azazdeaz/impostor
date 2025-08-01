
from pydantic import BaseModel, Field
import numpy as np
from scipy.spatial.transform import Rotation
from .transform_3d import Transform3D
import numpy.typing as npt

class Trail(BaseModel):
    start_point: npt.NDArray[np.float64] = Field(default_factory=lambda: np.zeros(3))
    start_direction: npt.NDArray[np.float64] = Field(default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    length: float = 1.0
    steps: int = 10

    class Config:
        arbitrary_types_allowed = True

    def transforms(self) -> list[Transform3D]:
        """Generate a list of Transform3D objects representing the trail."""
        transforms: list[Transform3D] = []
        step_length = self.length / self.steps
        for i in range(self.steps):
            position = self.start_point + i * step_length * self.start_direction
            rotation = Rotation.from_rotvec(np.cross(self.start_direction, np.array([0, 0, 1])))
            transforms.append(Transform3D(position=position, rotation=rotation, scale=np.ones(3)))
        return transforms
