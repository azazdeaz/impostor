from dataclasses import dataclass, field
from typing import Iterable
from scipy.spatial.transform._rotation import Rotation
from impostor.plant import Entity
import enum
import rerun as rr
from rerun.any_value import AnyBatchValue


class VascularType(enum.Enum):
    STEM = "stem"
    MERISTEM = "meristem"
    VEIN = "vein"


class MarkerComponent(rr.AsComponents):
    def as_component_batches(self) -> Iterable[rr.ComponentBatchLike]:
        return [AnyBatchValue(f"cmp.{self.__class__.__name__}", "✓")]


@dataclass
class Vascular(rr.AsComponents):
    length: float = 0.0
    radius: float = 0.04
    rotation: Rotation = Rotation.identity()
    type: VascularType = VascularType.STEM

    def as_component_batches(self) -> Iterable[rr.ComponentBatchLike]:
        return [
            AnyBatchValue("comps.VascularType.length", self.length),
            AnyBatchValue("comps.VascularType.radius", self.radius),
            AnyBatchValue("comps.VascularType.rotation", self.rotation.as_euler("xyz")),
            AnyBatchValue("comps.VascularType.type", str(self.type)),
        ]
    
@dataclass
class Mass(rr.AsComponents):
    mass: float = 1.0

    def as_component_batches(self) -> Iterable[rr.ComponentBatchLike]:
        return [AnyBatchValue("comps.Mass.mass", self.mass)]
    
@dataclass
class MassAbove(rr.AsComponents):
    mass: float = 0.0

    def as_component_batches(self) -> Iterable[rr.ComponentBatchLike]:
        return [AnyBatchValue("comps.MassAbove.mass", self.mass)]


@dataclass
class AxeNext:
    next: Entity


@dataclass
class AxePrev:
    prev: Entity


@dataclass
class Branches:
    branches: list[Entity] = field(default_factory=list)


@dataclass
class AttachmentOrientation:
    """Rotation around the growth direction of the parent stem."""

    azimuth: float = 0.0
    """0 means the branch grows the same direction as the parent stem, pi/2 means it grows perpendicular to the parent stem."""
    inclination: float = 0.0

    def as_rotation(self):
        return Rotation.from_euler("xyz", [self.inclination, 0, self.azimuth])

@dataclass
class BranchAttachment(MarkerComponent):
    """Marks the attachment of a branch to a stem."""

@dataclass
class LeafAttachment(MarkerComponent):
    """Marks the attachment of a leaf to a stem."""


@dataclass
class GrowthTip():
    """Marks the meristem tip of an axis."""
    branch_order: int = 0


@dataclass
class Root:
    pass
