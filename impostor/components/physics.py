from dataclasses import dataclass, field
from typing import Iterable

import rerun as rr
from rerun.any_value import AnyBatchValue
from scipy.spatial.transform._rotation import Rotation

from impostor.components import Entity, RigidTransformation


@dataclass
class Spring(rr.AsComponents):
    entity_a: Entity
    entity_b: Entity
    weight_a: float = 1.0
    weight_b: float = 0.0
    length: float = 0.0
    angle: Rotation = field(default_factory=Rotation.identity)
    angle_rest: Rotation = field(default_factory=Rotation.identity)
    angle_stiffness: float = 0.0 # 0.0 is no stiffness, 1.0 is full stiffness
    fixed_angle_stiffness: bool = False # If false, angle_stiffness is computed from mass and radius


    def to_transform(self) -> RigidTransformation:
        return RigidTransformation.from_rotation(self.angle).combine(RigidTransformation.from_z_translation(self.length))
    
    def as_component_batches(self) -> Iterable[rr.ComponentBatchLike]:
        return [
            AnyBatchValue("comps.Spring.entity_a", self.entity_a),
            AnyBatchValue("comps.Spring.entity_b", self.entity_b),
            AnyBatchValue("comps.Spring.length", self.length),
            AnyBatchValue("comps.Spring.angle", self.angle.as_euler("xyz")),
            AnyBatchValue("comps.Spring.angle_rest", self.angle_rest.as_euler("xyz")),
            AnyBatchValue("comps.Spring.angle_stiffness", self.angle_stiffness),
        ]