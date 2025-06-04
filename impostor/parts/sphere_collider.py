from dataclasses import dataclass

from .core import BasePart


@dataclass
class SphereCollider(BasePart):
    """A simple sphere collider for entities."""

    radius: float = 0.1
