from __future__ import annotations

from typing import List, Optional, Sequence, Union

import numpy as np
import rerun as rr
from pydantic import BaseModel, Field
from scipy.spatial.transform import Rotation

from ..engine import (
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
from ..leaf import LeafContext
from .mesh2d import Mesh2D
from .mesh3d import CompundMesh3D, Mesh3D
from ..transform_3d import Transform3D, Vector3

FORWARD = np.array([0.0, 0.0, 1.0])


class Turtle(Transform3D):
    pass


class NodeBlueprint(BaseModel):
    parent_index: int
    blueprint: Union[StemBlueprint, LeafBlueprint]


class StemBlueprint(BaseModel):
    transforms: List[Turtle] = Field(default_factory=lambda: [])
    nodes: List[NodeBlueprint] = Field(default_factory=lambda: [])
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


def generate_blueprint(
    symbols: Sequence[Symbol],
) -> StemBlueprint:
    turtle: Turtle = Turtle()
    transform_stack: List[Turtle] = []
    branch_point_stack: List[int] = []
    stack: List[StemBlueprint] = [StemBlueprint()]
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

        elif isinstance(symbol, BranchOpen):
            transform_stack.append(turtle.model_copy())
            parent = stack[-1]
            branch_point_stack.append(max(0, len(parent.transforms) - 1))
            stack.append(StemBlueprint())

        elif isinstance(symbol, BranchClose):
            if len(stack) > 1:
                complete_stem = stack.pop()
                branch_idx = branch_point_stack.pop()
                if current_leaf is not None:
                    if complete_stem is current_leaf.midrib:  # Leaf is done
                        stack[-1].nodes.append(
                            NodeBlueprint(
                                parent_index=branch_idx,
                                blueprint=current_leaf,
                            )
                        )
                        current_leaf = None
                    else:
                        current_leaf.veins.append(complete_stem)
                else:
                    stack[-1].nodes.append(
                        NodeBlueprint(
                            parent_index=branch_idx,
                            blueprint=complete_stem,
                        )
                    )
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
            turtle.rotation = local_euler(turtle.rotation, "y", symbol.angle)

        elif isinstance(symbol, Pitch):
            turtle.rotation = local_euler(turtle.rotation, "x", symbol.angle)

        elif isinstance(symbol, Roll):
            turtle.rotation = local_euler(turtle.rotation, "z", symbol.angle)

        elif isinstance(symbol, Tropism):
            turtle.rotation = apply_tropism(turtle.rotation, symbol.gravity, FORWARD)

        elif isinstance(symbol, MaterialKey):
            blueprint = stack[-1]
            blueprint.material_key = symbol.key
            if current_leaf is not None:
                current_leaf.material_key = symbol.key

    return stack[0]


def flatten_blueprint(root: StemBlueprint) -> List[StemBlueprint | LeafBlueprint]:
    """Flatten the blueprint tree into a list (backward compat for generate_mesh/log_transforms)."""
    result: List[StemBlueprint | LeafBlueprint] = [root]
    for node in root.nodes:
        if isinstance(node.blueprint, StemBlueprint):
            result.extend(flatten_blueprint(node.blueprint))
        else:
            result.append(node.blueprint)
    return result


def generate_blueprints(
    symbols: Sequence[Symbol],
) -> List[StemBlueprint | LeafBlueprint]:
    """Generate blueprints and return a flat list (backward compat wrapper)."""
    return flatten_blueprint(generate_blueprint(symbols))


def generate_mesh(blueprints: List[StemBlueprint | LeafBlueprint]) -> "CompundMesh3D":
    mesh3d = CompundMesh3D()

    for blueprint in blueprints:
        if isinstance(blueprint, StemBlueprint):
            profile = Mesh2D.circle(radius=1.0, segments=7)
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
