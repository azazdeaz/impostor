from dataclasses import dataclass, field
from typing import Iterable
from scipy.spatial.transform._rotation import Rotation
from impostor.plant import Entity
import rerun as rr
from rerun.any_value import AnyBatchValue

class MarkerComponent(rr.AsComponents):
    def as_component_batches(self) -> Iterable[rr.ComponentBatchLike]:
        return [AnyBatchValue(f"cmp.{self.__class__.__name__}", "✓")]


@dataclass
class AxeNext:
    next: Entity


@dataclass
class AxePrev:
    prev: Entity


@dataclass
class Attachments:
    attachments: list[Entity] = field(default_factory=list)


@dataclass
class AttachmentOrientation:
    """Rotation around the growth direction of the parent stem."""
    azimuth: float = 0.0
    """0 means the branch grows the same direction as the parent stem, pi/2 means it grows perpendicular to the parent stem."""
    inclination: float = 0.0

    def as_rotation(self):
        return Rotation.from_euler("yxz", [self.inclination, 0, self.azimuth])


@dataclass
class BranchAttachment(MarkerComponent):
    """Marks the attachment of a branch to a stem."""


@dataclass
class LeafAttachment(MarkerComponent):
    """Marks the attachment of a leaf to a stem."""


@dataclass
class VeinAttachment(MarkerComponent):
    """Marks the attachment of a vein to a midrib."""


@dataclass
class GrowthTip:
    """Marks the meristem tip of an axis."""

    branch_order: int = 0

@dataclass
class Label:
    label: str


@dataclass
class Root:
    pass
