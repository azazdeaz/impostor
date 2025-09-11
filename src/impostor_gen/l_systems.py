from collections.abc import Sequence
from typing import List, Optional
from pydantic import BaseModel, Field
import numpy as np
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


class F(Symbol):
    length: float = 1.0
    width: float = 1.0


class T(Symbol):
    """Tropism: lean toward a global (gravity) vector each time encountered."""

    gravity: float = 5.0  # degrees per application (max lean this step)
    # Optionally could add: vector: tuple[float, float, float] = (0.0, -1.0, 0.0)


class Yaw(Symbol):
    angle: float = 25.0  # degrees (positive = left, negative = right)


class Pitch(Symbol):
    angle: float = 25.0  # degrees (positive = down, negative = up)


class Roll(Symbol):
    angle: float = 25.0  # degrees (positive = CCW looking forward, negative = CW)


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
            return Symbol()
        return self.world[index]


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
    def _apply_tropism(rotation: Rotation, max_degrees: float,
                       forward: np.ndarray,
                       gravity_vec: np.ndarray = np.array([0.0, -1.0, 0.0])) -> Rotation:
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

    def interpret(self) -> Mesh3D:
        turtle = Transform3D()
        transforms: List[Transform3D] = [turtle.model_copy()]

        for symbol in self.world:
            if isinstance(symbol, F):
                # Move forward
                direction = turtle.rotation.apply(self.forward)
                turtle.position = turtle.position + direction * symbol.length
                transforms.append(turtle.model_copy())

            elif isinstance(symbol, Yaw):
                # Positive angle = left turn; user semantic +(a)=right so you'd create Yaw(angle=-a) for '+'
                turtle.rotation = self._local_euler(turtle.rotation, 'y', symbol.angle)

            elif isinstance(symbol, Pitch):
                # Positive angle = pitch down
                turtle.rotation = self._local_euler(turtle.rotation, 'x', symbol.angle)

            elif isinstance(symbol, Roll):
                # Positive angle = counter-clockwise looking forward
                turtle.rotation = self._local_euler(turtle.rotation, 'z', symbol.angle)

            elif isinstance(symbol, T):
                turtle.rotation = self._apply_tropism(
                    turtle.rotation, symbol.gravity, self.forward
                )

            # (Optional) handle Stem or other symbols (could change width, spawn geometry, etc.)

        profile = Mesh2D.circle(radius=0.4, segments=7)

        mesh3d = extrude_mesh2d_along_points(profile, transforms)
        return mesh3d
