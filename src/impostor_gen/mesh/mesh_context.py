from __future__ import annotations

from typing import TYPE_CHECKING, Any

from ..engine.context import ContextSymbol

if TYPE_CHECKING:
    from .mesh3d import Mesh3D


class MeshContext(ContextSymbol):
    """Base class for mesh generation contexts placed into the L-system.

    Subclass and override generate_visual / generate_collider to provide
    concrete mesh generation for stems, leaves, or custom geometry.
    """

    def generate_visual(self, blueprint: Any) -> Mesh3D:
        raise NotImplementedError

    def generate_collider(self, blueprint: Any) -> Mesh3D:
        raise NotImplementedError
