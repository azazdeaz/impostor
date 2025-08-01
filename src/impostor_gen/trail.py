
from pydantic import BaseModel, Field
import numpy as np
from scipy.spatial.transform import Rotation
from .shapes import Transform

class Trail(BaseModel):
    start_point: np.ndarray = Field(default_factory=lambda: np.zeros(3))
    start_direction: np.ndarray = Field(default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    length: float = 1.0
    steps: int = 10

    def transforms(self) -> list[Transform]:
        """Generate a list of Transform objects representing the trail."""
        transforms: list[Transform] = []
        step_length = self.length / self.steps
        for i in range(self.steps):
            position = self.start_point + i * step_length * self.start_direction
            rotation = Rotation.from_rotvec(np.cross(self.start_direction, np.array([0, 0, 1])))
            transforms.append(Transform(position=position, rotation=rotation, scale=np.ones(3)))
        return transforms
