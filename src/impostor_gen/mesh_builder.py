from typing import List, Optional, Sequence

import numpy as np
import rerun as rr
from pydantic import BaseModel, Field
from scipy.spatial.transform import Rotation

from impostor_gen.engine.core_symbols import UV

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
from .transform_3d import Transform3D, Vector3

FORWARD = np.array([0.0, 0.0, 1.0])


class Turtle(Transform3D):
    uv: UV = Field(default_factory=lambda: UV())


class StemBlueprint(BaseModel):
    transforms: List[Turtle] = Field(default_factory=lambda: [])
    material_key: Optional[str] = None


class LeafBlueprint(BaseModel):
    midrib: StemBlueprint
    veins: List[StemBlueprint] = Field(default_factory=lambda: [])
    material_key: Optional[str] = None

    @property
    def transforms(self) -> List[Turtle]:
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
    turtle: Turtle = Turtle()
    transform_stack: List[Turtle] = []
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

        elif isinstance(symbol, Diameter):
            turtle.set_scale(symbol.diameter)

        elif isinstance(symbol, UV):
            turtle.uv = symbol.model_copy()

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
            # This expects that all the lateral veins have the same number of transforms
            midrib_div = len(blueprint.midrib.transforms)

            layers: List[List[np.ndarray]] = []

            assert len(blueprint.veins) == (midrib_div - 1) * 2, (
                f"Each midrib section must have two secondary veins except the tip. Found {len(blueprint.veins)} veins for {midrib_div} midrib sections."
            )

            for i in range(midrib_div):
                if i == midrib_div - 1:
                    layers.append([blueprint.midrib.transforms[i].position])
                else:
                    layers.append(
                        [t.position for t in blueprint.veins[i * 2].transforms[::-1]]
                        + [blueprint.midrib.transforms[i].position]
                        + [t.position for t in blueprint.veins[i * 2 + 1].transforms]
                    )

            vertices = np.concatenate(layers)
            one_start = 0
            faces: List[List[int]] = []
            is_closed = False
            winding_clockwise = True

            for i in range(len(layers) - 1):
                two_start = one_start + len(layers[i])
                one_id = 0
                two_id = 0
                one_size = len(layers[i])
                two_size = len(layers[i + 1])

                id_margin = 0 if is_closed else 1
                while one_id < one_size - id_margin or two_id < two_size - id_margin:
                    id1 = one_start + one_id % one_size
                    id2 = two_start + two_id % two_size
                    id1_next = one_start + (one_id + 1) % one_size
                    id2_next = two_start + (two_id + 1) % two_size

                    # the positions of the vertices on the layer from 0 to 1
                    up_progress = one_id / one_size
                    down_progress = two_id / two_size

                    if up_progress > down_progress:
                        if winding_clockwise:
                            new_face = [id1, id2_next, id2]
                        else:
                            new_face = [id1, id2, id2_next]
                        two_id += 1
                    else:
                        if winding_clockwise:
                            new_face = [id1, id1_next, id2]
                        else:
                            new_face = [id1, id2, id1_next]
                        one_id += 1

                    # Add the face to the list of faces
                    faces.append(new_face)

                one_start = two_start

            leaf_mesh = Mesh3D(
                vertex_positions=vertices,
                # vertex_texcoords=uv_grid.reshape(-1, 2),
                triangle_indices=np.array(faces),
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
