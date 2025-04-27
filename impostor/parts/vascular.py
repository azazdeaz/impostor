from dataclasses import dataclass, field
from typing import Iterable
from scipy.spatial.transform._rotation import Rotation
from impostor.parts.core import BasePart
import impostor.parts as parts
from impostor.plant import Entity
import enum
import rerun as rr
from rerun.any_value import AnyBatchValue


class VascularType(enum.Enum):
    STEM = "stem"
    MIDRIB = "midrib"
    VEIN = "vein"

@dataclass
class Vascular(rr.AsComponents, BasePart):
    length: float = 0.0
    radius: float = 0.04
    rotation: Rotation = Rotation.identity()
    type: VascularType = VascularType.STEM

    def step(self, plant, entity):
        growth_plan = plant.get_components(entity).get_by_type(parts.GrowthPlan)
        if growth_plan is None:
            return
        
        self.length = growth_plan.get_length()
        self.radius = growth_plan.get_radius()
        # self.rotation = growth_plan.get_rotation()

    def as_component_batches(self) -> Iterable[rr.ComponentBatchLike]:
        return [
            AnyBatchValue("comps.VascularType.length", self.length),
            AnyBatchValue("comps.VascularType.radius", self.radius),
            AnyBatchValue("comps.VascularType.rotation", rr.components.Vector3D(self.rotation.as_euler("xyz"))),
            AnyBatchValue("comps.VascularType.type", str(self.type)),
        ]