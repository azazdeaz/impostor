from typing import List, Optional, Sequence

import numpy as np
import rerun as rr
from pydantic import BaseModel, Field
from scipy.spatial.transform import Rotation

from .engine import (
    BranchClose,
    BranchOpen,
    Diameter,
    F,
    MaterialKey,
    Pitch,
    Roll,
    StemContext,
    Symbol,
    Tropism,
    Yaw,
)
from .extrude import extrude_mesh2d_along_points
from .leaf import LeafContext
from .mesh2d import Mesh2D
from .mesh3d import CompundMesh3D, Mesh3D
from .transform_3d import Transform3D

FORWARD = np.array([0.0, 0.0, 1.0])


class StemBlueprint(BaseModel):
    transforms: List[Transform3D] = Field(default_factory=lambda: [])
    material_key: Optional[str] = None


class LeafBlueprint(BaseModel):
    midrib: StemBlueprint
    veins: List[StemBlueprint] = Field(default_factory=lambda: [])
    material_key: Optional[str] = None

    @property
    def transforms(self) -> List[Transform3D]:
        return self.midrib.transforms + [
            t for vein in self.veins for t in vein.transforms
        ]


def local_euler(rotation: Rotation, axis: str, degrees: float) -> Rotation:
    """Return updated rotation after applying a local-axis Euler rotation."""
    if abs(degrees) < 1e-9:
        return rotation
    delta = Rotation.from_euler(axis, degrees, degrees=True)
    # Local axis -> post-multiply
    return rotation * delta


def apply_tropism(
    rotation: Rotation,
    max_degrees: float,
    forward: np.ndarray,
    gravity_vec: np.ndarray = np.array([0.0, -1.0, 0.0]),
) -> Rotation:
    """Lean heading toward gravity vector by up to max_degrees (world-axis application)."""
    if max_degrees <= 0.0:
        return rotation

    f = rotation.apply(forward)
    g = gravity_vec.astype(np.float64)
    g_norm = np.linalg.norm(g)
    if g_norm < 1e-9:
        return rotation
    g /= g_norm

    dot = np.clip(np.dot(f, g), -1.0, 1.0)
    theta = np.arccos(dot)
    if theta < 1e-6:
        return rotation  # already aligned

    # Axis of rotation (world)
    axis = np.cross(f, g)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-9:
        return rotation
    axis /= axis_norm

    lean_rad = np.deg2rad(min(max_degrees, np.rad2deg(theta)))
    rot_world = Rotation.from_rotvec(axis * lean_rad)
    # World-axis rotation -> pre-multiply
    return rot_world * rotation


def generate_blueprints(
    symbols: Sequence[Symbol],
) -> List[StemBlueprint | LeafBlueprint]:
    turtle = Transform3D()
    transform_stack: List[Transform3D] = []
    closed_branches: List[StemBlueprint] = []
    stack: List[StemBlueprint] = [StemBlueprint()]
    finished_leaves: List[LeafBlueprint] = []
    current_leaf: Optional[LeafBlueprint] = None

    for symbol in symbols:
        if isinstance(symbol, F):
            # Move forward
            direction = turtle.rotation.apply(FORWARD)
            turtle.position = turtle.position + direction * symbol.length

            blueprint = stack[-1]
            blueprint.transforms.append(turtle.model_copy())

        if isinstance(symbol, Diameter):
            turtle.set_scale(symbol.diameter)

        elif isinstance(symbol, BranchOpen):
            transform_stack.append(turtle.model_copy())
            stack.append(StemBlueprint())

        elif isinstance(symbol, BranchClose):
            if len(stack) > 1:
                complete_stem = stack.pop()
                if current_leaf is not None:
                    if complete_stem is current_leaf.midrib:  # Leaf is done
                        finished_leaves.append(current_leaf)
                        current_leaf = None
                    else:
                        current_leaf.veins.append(complete_stem)
                else:
                    closed_branches.append(complete_stem)
                turtle = transform_stack.pop()
            else:
                raise ValueError("Unmatched BranchClose symbol encountered.")

        elif isinstance(symbol, StemContext):
            blueprint = stack[-1]
            blueprint.transforms.append(turtle.model_copy())

        elif isinstance(symbol, LeafContext):
            if current_leaf is not None:
                raise ValueError("Nested leaves are not supported.")
            current_leaf = LeafBlueprint(midrib=stack[-1])

        elif isinstance(symbol, Yaw):
            # Positive angle = left turn; user semantic +(a)=right so you'd create Yaw(angle=-a) for '+'
            turtle.rotation = local_euler(turtle.rotation, "y", symbol.angle)

        elif isinstance(symbol, Pitch):
            # Positive angle = pitch down
            turtle.rotation = local_euler(turtle.rotation, "x", symbol.angle)

        elif isinstance(symbol, Roll):
            # Positive angle = counter-clockwise looking forward
            turtle.rotation = local_euler(turtle.rotation, "z", symbol.angle)

        elif isinstance(symbol, Tropism):
            turtle.rotation = apply_tropism(turtle.rotation, symbol.gravity, FORWARD)

        elif isinstance(symbol, MaterialKey):
            blueprint = stack[-1]
            blueprint.material_key = symbol.key
            if current_leaf is not None:
                current_leaf.material_key = symbol.key

    return stack + closed_branches + finished_leaves


def generate_mesh(blueprints: List[StemBlueprint | LeafBlueprint]) -> "CompundMesh3D":
    profile = Mesh2D.circle(radius=0.4, segments=7)

    mesh3d = CompundMesh3D()

    for blueprint in blueprints:
        if isinstance(blueprint, StemBlueprint):
            # Ensure we have at least two transforms
            if len(blueprint.transforms) >= 2:
                stem_mesh = extrude_mesh2d_along_points(profile, blueprint.transforms)
                stem_mesh.material_key = blueprint.material_key
                mesh3d = mesh3d.merge(stem_mesh)

        else:  # LeafBlueprint
            # assert len(blueprint.midrib.transforms) == len(blueprint.veins) * 2, f"Midrib transforms: {len(blueprint.midrib.transforms)}, Veins: {len(blueprint.veins) * 2}"
            midrib_div = len(blueprint.midrib.transforms)
            sec_vein_div = len(blueprint.veins[0].transforms)
            horisontal_div = sec_vein_div * 2 + 1
            vertex_grid = np.zeros((midrib_div, horisontal_div, 3))
            u = np.linspace(0.0, 1.0, horisontal_div)
            v = np.linspace(0.0, 1.0, midrib_div)
            u_grid, v_grid = np.meshgrid(u, v)
            uv_grid = np.stack((u_grid, v_grid), axis=-1)
            for i in range(midrib_div):
                vertex_grid[i, :sec_vein_div, :] = [
                    t.position for t in blueprint.veins[i * 2].transforms[::-1]
                ]
                vertex_grid[i, sec_vein_div, :] = blueprint.midrib.transforms[
                    i
                ].position
                vertex_grid[i, sec_vein_div + 1 :, :] = [
                    t.position for t in blueprint.veins[i * 2 + 1].transforms
                ]
            # Create faces
            faces = np.zeros(
                ((midrib_div - 1) * (horisontal_div - 1) * 2, 3), dtype=np.int32
            )
            for i in range(midrib_div - 1):
                for j in range(horisontal_div - 1):
                    v0 = i * horisontal_div + j
                    v1 = v0 + 1
                    v2 = v0 + horisontal_div
                    v3 = v2 + 1
                    faces[(i * (horisontal_div - 1) + j) * 2 + 0, :] = [v0, v2, v1]
                    faces[(i * (horisontal_div - 1) + j) * 2 + 1, :] = [v1, v2, v3]

            leaf_mesh = Mesh3D(
                vertex_positions=vertex_grid.reshape(-1, 3),
                vertex_texcoords=uv_grid.reshape(-1, 2),
                triangle_indices=faces,
                material_key=blueprint.material_key,
            )
            mesh3d = mesh3d.merge(leaf_mesh)
    return mesh3d


def log_transforms(blueprints: List[StemBlueprint | LeafBlueprint]):
    arrows = rr.Arrows3D(
        vectors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
    )
    rr.log("stem/frames", rr.Clear(recursive=True))
    for i, b in enumerate(blueprints):
        for j, t in enumerate(b.transforms):
            rr.log(f"stem/frames/{i}/{j}", t.to_rerun(), arrows)
