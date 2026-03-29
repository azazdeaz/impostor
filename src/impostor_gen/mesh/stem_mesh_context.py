from __future__ import annotations

from typing import Any

from .extrude import extrude_mesh2d_along_points
from .mesh_context import MeshContext
from .mesh2d import Mesh2D
from .mesh3d import Mesh3D


class StemMeshContext(MeshContext):
    """Generates extruded tube meshes for stem blueprints."""

    segments: int = 7

    def generate_visual(self, blueprint: Any) -> Mesh3D:
        profile = Mesh2D.circle(radius=1.0, segments=self.segments)
        if len(blueprint.transforms) < 2:
            return Mesh3D.empty()
        mesh = extrude_mesh2d_along_points(profile, blueprint.transforms)
        mesh.material_key = blueprint.material_key
        return mesh

    def generate_collider(self, blueprint: Any) -> Mesh3D:
        return self.generate_visual(blueprint)
