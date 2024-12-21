from dataclasses import dataclass, field

from scipy.spatial.transform._rotation import Rotation

from impostor.components import Entity, RigidTransformation

@dataclass
class Spring:
    entity_a: Entity
    entity_b: Entity
    weight_a: float = 1.0
    weight_b: float = 1.0
    length: float = 0.0
    angle: Rotation = field(default_factory=Rotation.identity)
    angle_stiffness: float = 0.0


    def to_transform(self) -> RigidTransformation:
        return RigidTransformation.from_rotation(self.angle).combine(RigidTransformation.from_z_translation(self.length))