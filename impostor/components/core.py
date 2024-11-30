from dataclasses import dataclass, field
from scipy.spatial.transform._rotation import Rotation
from impostor.plant import Entity
import rerun as rr

@dataclass
class Stem:
    length: float = 0.0
    radius: float = 0.04
    rotation: Rotation = Rotation.identity()


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
class Branch:
    """Rotation around the growth direction of the parent stem."""
    azimuth: float = 0.0
    """0 means the branch grows the same direction as the parent stem, pi/2 means it grows perpendicular to the parent stem."""
    inclination: float = 0.0,


@dataclass
class Root:
    pass