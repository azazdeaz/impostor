from collections.abc import Sequence
from typing import Callable, Generic, List, Optional, Tuple, TypeVar
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


class Tropism(Symbol):
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




class Rule(BaseModel):
    def apply(self, writer: Writer):
        pass


T = TypeVar('T', bound=Symbol)

class BasicRule(Rule, Generic[T]):
    left: type[T]  # This ensures left is a subclass of Symbol
    right: List[Symbol] | Callable[[T], List[Symbol]]  # This ensures the callable accepts an instance of the specific subclass

    def apply(self, writer: Writer):
        current = writer.peek(0)
        if isinstance(current, self.left):
            if callable(self.right):
                # The type checker now understands that current is of type T
                writer.write(self.right(current))  
            else:
                writer.write(self.right)


# ---------------- L-System ---------------- #


class LSystem(BaseModel):
    world: Sequence[Symbol]
    rules: Sequence[Rule]

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
