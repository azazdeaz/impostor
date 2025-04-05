from dataclasses import dataclass, field
from typing import Iterable

import numpy as np
import rerun as rr
from rerun.any_value import AnyBatchValue
from scipy.spatial.transform import Rotation, Slerp

from impostor.parts.core import BasePart

@dataclass
class GrowthPlan(rr.AsComponents, BasePart):
    state: float = 0.0
    growth_speed: float = 0.004
    length_start: float = 0.0
    length_end: float = 0.0
    radius_start: float = 0.0
    radius_end: float = 0.0
    rotation_start: Rotation = field(default_factory=Rotation.identity)
    rotation_end: Rotation = field(default_factory=Rotation.identity)

    def get_length_at(self, t: float) -> float:
        return self.length_start + (self.length_end - self.length_start) * t

    def get_rotation_at(self, t: float) -> Rotation:
        t = max(0, min(1, t))
        slerp = Slerp([0, 1], Rotation.concatenate([self.rotation_start, self.rotation_end]))
        return slerp(t)
    
    def get_radius_at(self, t: float) -> float:
        return self.radius_start + (self.radius_end - self.radius_start) * t
    
    def get_length(self) -> float:
        return self.get_length_at(self.state)
    
    def get_radius(self) -> float:
        return self.get_radius_at(self.state)
    
    def get_rotation(self) -> Rotation:
        return self.get_rotation_at(self.state)
    
    def step(self, plant, entity):
        step = np.minimum(self.growth_speed, (1.0 - self.state) / 6) # NOTE: Hacky way to keep all the parts of the plant moving a bit
        self.state = np.minimum(1.0, self.state + step)

    def as_component_batches(self) -> Iterable[rr.ComponentBatchLike]:
        return [
            AnyBatchValue("> GrowthPlan.state", self.state),
            AnyBatchValue("> GrowthPlan.length_start", self.length_start),
            AnyBatchValue("> GrowthPlan.length_end", self.length_end),
            AnyBatchValue("> GrowthPlan.rotation_start", self.rotation_start.as_euler("xyz")),
            AnyBatchValue("> GrowthPlan.rotation_end", self.rotation_end.as_euler("xyz")),
            AnyBatchValue("> GrowthPlan.radius_start", self.radius_start),
            AnyBatchValue("> GrowthPlan.radius_end", self.radius_end),
        ]