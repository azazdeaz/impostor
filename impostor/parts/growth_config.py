from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, List, Self

import numpy as np
import rerun as rr
from rerun.any_value import AnyBatchValue
from scipy.spatial.transform import Rotation, Slerp
from enum import Enum
import impostor.parts as parts

from impostor.parts.core import BasePart
from impostor.utils import Curve

@dataclass
class GrowthParams(BasePart):
    params: List[(BasePart, str, Callable)] = field(default_factory=dict)
    def step(self, plant, entity):
        


@dataclass
class GrowthConfig(rr.AsComponents, BasePart):
    class Role(Enum):
        STEM = "stem"

    aging_rate: float = 0.01
    age: float = 0.0
    petiole_no: float = 0.0
    divisions: int = 10

    def petiole_bend(self) -> int:
        return Curve([(0.0, 1.0), (1.0, 8.0)], Curve.Method.MAKIMA).evaluate(self.age)
    
    # def petiole_length(self) -> float:


    def step(self, plant, entity):
        self.age += self.aging_rate

        stem = plant.get_components(entity).get_by_type(parts.Vascular)
        if stem is None:
            return

        # stem.length = self.get_length()
        # stem.radius = self.get_radius()
        stem.rotation = Rotation.from_euler(
            "xyz", [self.petiole_bend(), 0, 0], degrees=True
        )
