from dataclasses import dataclass, field
from typing import Iterable

import rerun as rr
from rerun.any_value import AnyBatchValue
from scipy.spatial.transform import Rotation, Slerp

from impostor.components import Entity
import impostor.components as comp
from impostor.utils import Curve


@dataclass
class LeafMeta(rr.AsComponents):
    attachment_parent_entity: Entity

    midrib_entitiy_count: int = 12
    midrib_length: float = 0.6
    lateral_vein_count: int = 5
    lateral_vein_length: float = 0.32
    lateral_vein_entitiy_count: int = 5
    vein_length_multiplier: Curve | None = None
    attacment_orientation: comp.AttachmentOrientation = field(default_factory=comp.AttachmentOrientation)

    base_entity: Entity | None = None
    midrib_entities: list[Entity] = field(default_factory=list)
    lateral_vein_entities: list[Entity] = field(default_factory=list)
    lateral_vein_bases_left: list[Entity] = field(default_factory=list)
    lateral_vein_bases_right: list[Entity] = field(default_factory=list)

    growth_stage: float = 0.0

    def as_component_batches(self) -> Iterable[rr.ComponentBatchLike]:
        return [
            AnyBatchValue("comps.LeafMeta.growth_stage", self.growth_stage),
            AnyBatchValue("comps.LeafMeta.midrib_entitiy_count", self.midrib_entitiy_count),

        ]

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
    
    def as_component_batches(self) -> Iterable[rr.ComponentBatchLike]:
        return [
            AnyBatchValue("comps.GrowthPlan.length_start", self.length_start),
            AnyBatchValue("comps.GrowthPlan.length_end", self.length_end),
            AnyBatchValue("comps.GrowthPlan.rotation_start", self.rotation_start.as_euler("xyz")),
            AnyBatchValue("comps.GrowthPlan.rotation_end", self.rotation_end.as_euler("xyz")),
        ]