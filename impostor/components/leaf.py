from dataclasses import dataclass, field
from typing import Iterable

import rerun as rr
from rerun.any_value import AnyBatchValue
from scipy.spatial.transform import Rotation, Slerp

from impostor.components import Entity
from impostor.utils import Curve


@dataclass
class LeafMeta(rr.AsComponents):
    attachment_parent_entity: Entity

    midrib_entitiy_count: int = 12
    midrib_length: float = 0.7
    lateral_vein_count: int = 5
    lateral_vein_length: float = 0.42
    lateral_vein_entitiy_count: int = 5
    vein_length_multiplier: Curve | None = None

    base_entity: Entity | None = None
    midrib_entities: list[Entity] = field(default_factory=list)
    lateral_vein_entities: list[Entity] = field(default_factory=list)
    lateral_vein_bases_left: list[Entity] = field(default_factory=list)
    lateral_vein_bases_right: list[Entity] = field(default_factory=list)

    growth_stage: float = 0.0

    def as_component_batches(self) -> Iterable[rr.ComponentBatchLike]:
        return [AnyBatchValue("comps.LeafMera.growth_stage", self.growth_stage)]

@dataclass
class GrowthPlan(rr.AsComponents):
    length_start: float = 0.0
    length_end: float = 0.0
    rotation_start: Rotation = field(default_factory=Rotation.identity)
    rotation_end: Rotation = field(default_factory=Rotation.identity)

    def get_length_at(self, t: float) -> float:
        return self.length_start + (self.length_end - self.length_start) * t

    def get_rotation_at(self, t: float) -> Rotation:
        t = max(0, min(1, t))
        slerp = Slerp([0, 1], Rotation.concatenate([self.rotation_start, self.rotation_end]))
        return slerp(t)