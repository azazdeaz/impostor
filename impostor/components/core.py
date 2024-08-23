from dataclasses import dataclass, field
from scipy.spatial.transform._rotation import Rotation
from impostor.plant import Entity

@dataclass
class Stem:
    length: float = 0.0
    radius: float = 0.1
    rotation: Rotation = Rotation.identity()


@dataclass
class AxeNext:
    next: Entity


@dataclass
class AxePrev:
    prev: Entity


@dataclass
class Root:
    pass