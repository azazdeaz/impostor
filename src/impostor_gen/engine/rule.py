from collections.abc import Sequence
from pydantic import BaseModel
from typing import Callable, Generic, List, Optional, TypeVar, TYPE_CHECKING

from .symbol import Symbol

if TYPE_CHECKING:
    from .context import Context


from abc import ABC, abstractmethod


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


class Rule(ABC):
    @abstractmethod
    def apply(self, writer: Writer, context: "Context") -> None:
        pass


T = TypeVar("T", bound=Symbol)


class BasicRule(BaseModel, Rule, Generic[T]):
    left: type[T]  # This ensures left is a subclass of Symbol
    right: (
        List[Symbol] | Callable[[T], List[Symbol]]
    )  # This ensures the callable accepts an instance of the specific subclass

    def apply(self, writer: Writer, context: "Context"):
        current = writer.peek(0)
        if isinstance(current, self.left):
            if callable(self.right):
                # The type checker now understands that current is of type T
                writer.write(self.right(current))
            else:
                writer.write(self.right)
