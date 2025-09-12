from collections.abc import Sequence
from typing import List, Optional, Tuple
from pydantic import BaseModel, Field
import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation
from .transform_3d import Transform3D
from .extrude import extrude_mesh2d_along_points
from .mesh2d import Mesh2D
from .mesh3d import Mesh3D


# ---------------- Symbols ---------------- #


class Symbol(BaseModel):
    pass


class Stem(Symbol):
    cross_sections: int = 3
    divisions: int = 5

    def __str__(self) -> str:
        return f"Stem({self.cross_sections}, {self.divisions})"


class F(Symbol):
    length: float = 1.0
    width: float = 1.0

    def __str__(self) -> str:
        return f"F({self.length:.2f}, {self.width:.2f})"


class T(Symbol):
    """Tropism: lean toward a global (gravity) vector each time encountered."""

    gravity: float = 5.0  # degrees per application (max lean this step)

    def __str__(self) -> str:
        return f"T({self.gravity:.1f})"


class Yaw(Symbol):
    angle: float = 25.0  # degrees (positive = left, negative = right)

    def __str__(self) -> str:
        return f"Yaw({self.angle:.1f})"


class Pitch(Symbol):
    angle: float = 25.0  # degrees (positive = down, negative = up)

    def __str__(self) -> str:
        return f"Pitch({self.angle:.1f})"


class Roll(Symbol):
    angle: float = 25.0  # degrees (positive = CCW looking forward, negative = CW)

    def __str__(self) -> str:
        return f"Roll({self.angle:.1f})"


class BranchOpen(Symbol):
    def __str__(self) -> str:
        return "BranchOpen"


class BranchClose(Symbol):
    def __str__(self) -> str:
        return "BranchClose"

class Tip(Symbol):
    order: int = 0 # Default to order 0 (main trunk)
    def __str__(self) -> str:
        return "Tip"

# ---------------- Rewriting Infra ---------------- #


class Writer(BaseModel):
    world: Sequence[Symbol]
    pointer: int
    window: int = 1
    replacement: Optional[Sequence[Symbol]] = None

    def extend_window(self):
        self.window += 1

    def write(self, replacement: Sequence[Symbol]):
        self.replacement = replacement

    def peek(self, offset: int) -> Symbol:
        index = self.pointer + offset
        if index < 0 or index >= len(self.world):
            raise IndexError(
                f"Peek index {index} out of bounds for world of size {len(self.world)}"
            )
        return self.world[index]


class StemBlueprint(BaseModel):
    transforms: List[Transform3D] = Field(default_factory=lambda: [])
    radii: List[float] = Field(default_factory=lambda: [])
    cross_sections: int = 0
    divisions: int = 6


class Rule(BaseModel):
    def apply(self, writer: Writer):
        pass


class BasicRule(Rule):
    left: type[Symbol]
    right: List[Symbol]

    def apply(self, writer: Writer):
        if isinstance(writer.peek(0), self.left):
            writer.write(self.right)


# ---------------- L-System ---------------- #


class LSystem(BaseModel):
    world: Sequence[Symbol]
    rules: Sequence[Rule]
    forward: np.ndarray = Field(default_factory=lambda: np.array([0.0, 0.0, 1.0]))

    class Config:
        arbitrary_types_allowed = True

    def iterate(self, n: int = 1):
        for _ in range(n):
            pointer = 0
            new_world: List[Symbol] = []
            while pointer < len(self.world):
                for rule in self.rules:
                    writer = Writer(world=self.world, pointer=pointer)
                    rule.apply(writer)
                    if writer.replacement is not None:
                        new_world.extend(writer.replacement)
                        pointer += writer.window
                        break
                else:
                    new_world.append(self.world[pointer])
                    pointer += 1
            self.world = new_world

    # ------------- Helper rotation methods ------------- #

    @staticmethod
    def _local_euler(rotation: Rotation, axis: str, degrees: float) -> Rotation:
        """Return updated rotation after applying a local-axis Euler rotation."""
        if abs(degrees) < 1e-9:
            return rotation
        delta = Rotation.from_euler(axis, degrees, degrees=True)
        # Local axis -> post-multiply
        return rotation * delta

    @staticmethod
    def _apply_tropism(
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

    # ------------- Interpretation ------------- #

    def generate_blueprints(self) -> List[StemBlueprint]:
        turtle = Transform3D()
        transform_stack: List[Transform3D] = []
        closed_branches: List[StemBlueprint] = []
        stack: List[StemBlueprint] = [StemBlueprint()]

        for symbol in self.world:
            if isinstance(symbol, F):
                # Move forward
                direction = turtle.rotation.apply(self.forward)
                turtle.position = turtle.position + direction * symbol.length
                blueprint = stack[-1]
                blueprint.transforms.append(turtle.model_copy())
                blueprint.radii.append(symbol.width)

            elif isinstance(symbol, BranchOpen):
                transform_stack.append(turtle.model_copy())
                stack.append(StemBlueprint())

            elif isinstance(symbol, BranchClose):
                if len(stack) > 1:
                    closed_branches.append(stack.pop())
                    turtle = transform_stack.pop()
                else:
                    raise ValueError("Unmatched BranchClose symbol encountered.")

            elif isinstance(symbol, Stem):
                blueprint = stack[-1]
                blueprint.cross_sections = symbol.cross_sections
                blueprint.divisions = symbol.divisions
                blueprint.transforms.append(turtle.model_copy())
                blueprint.radii.append(1.0)  # TODO start with the parent width

            elif isinstance(symbol, Yaw):
                # Positive angle = left turn; user semantic +(a)=right so you'd create Yaw(angle=-a) for '+'
                turtle.rotation = self._local_euler(turtle.rotation, "y", symbol.angle)

            elif isinstance(symbol, Pitch):
                # Positive angle = pitch down
                turtle.rotation = self._local_euler(turtle.rotation, "x", symbol.angle)

            elif isinstance(symbol, Roll):
                # Positive angle = counter-clockwise looking forward
                turtle.rotation = self._local_euler(turtle.rotation, "z", symbol.angle)

            elif isinstance(symbol, T):
                turtle.rotation = self._apply_tropism(
                    turtle.rotation, symbol.gravity, self.forward
                )

        return stack + closed_branches
    
    def generate_mesh(self, blueprints: List[StemBlueprint]) -> Mesh3D:
        profile = Mesh2D.circle(radius=0.4, segments=7)

        mesh3d = Mesh3D.empty()
        for b in enumerate(blueprints):
            print(f"Branch {b[0]}: {len(b[1].transforms)} segments")

        for blueprint in blueprints:
            if len(blueprint.transforms) >= 2:
                # Ensure we have at least two transforms and two radii to create a stem
                if len(blueprint.radii) != len(blueprint.transforms):
                    raise ValueError(
                        "Number of radii must match number of transforms in a StemBlueprint."
                    )
                stem_mesh = extrude_mesh2d_along_points(profile, blueprint.transforms)
                mesh3d = mesh3d.merge(stem_mesh)
        
        return mesh3d

    def log_mesh(self, blueprints: List[StemBlueprint]):
        mesh3d = self.generate_mesh(blueprints)
        rr.log("extruded_mesh", mesh3d.to_rerun())

    def log_transforms(self, blueprints: List[StemBlueprint]):
        arrows = rr.Arrows3D(
            vectors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
        )
        for i, b in enumerate(blueprints):
            for j, t in enumerate(b.transforms):
                rr.log(f"stem/{i}/{j}", t.to_rerun(), arrows)
        

    def log_graph(self):
        node_ids = [str(i) for i in range(len(self.world))]
        node_labels = [str(symbol) for symbol in self.world]
        edges: List[Tuple[str, str]] = []

        branch_stack: List[int] = []

        curr_id = 0
        for next_id, next_symbol in enumerate(self.world[1:], start=1):
            if isinstance(next_symbol, BranchOpen):
                branch_stack.append(curr_id)

            edges.append((node_ids[curr_id], node_ids[next_id]))
            curr_id = next_id

            if isinstance(next_symbol, BranchClose):
                if branch_stack:
                    curr_id = branch_stack.pop()

        rr.log(
            "world_graph",
            rr.GraphNodes(node_ids=node_ids, labels=node_labels),
            rr.GraphEdges(
                edges=edges,
                graph_type="directed",
            ),
        )

    def log_as_markdown(self):
        tab_size = 0
        markdown_lines: List[str] = []
        for symbol in self.world:
            if isinstance(symbol, BranchClose):
                tab_size = max(0, tab_size - 1)

            indent = "  " * tab_size
            markdown_lines.append(f"{indent}- {str(symbol)}")

            if isinstance(symbol, BranchOpen):
                tab_size += 1

        rr.log(
            "markdown",
            rr.TextDocument(
                "\n".join(markdown_lines),
                media_type=rr.MediaType.MARKDOWN,
            ),
        )
