from typing import Any, Optional, TypeVar

from pydantic import BaseModel

from .branch_symbols import BranchOpen
from .symbol import Symbol
from .branch_symbols import BranchClose


class ContextSymbol(Symbol):
    def get_context(self) -> Any:
        pass


T_Context = TypeVar("T_Context")


class Context(BaseModel):
    _context_stack: list[dict[type, Any]] = [{}]

    def feed_node(self, node: Any) -> None:
        if isinstance(node, BranchOpen):
            self._context_stack.append(self._context_stack[-1].copy())
        elif isinstance(node, BranchClose):
            if len(self._context_stack) > 1:
                self._context_stack.pop()
            else:
                raise ValueError("Unmatched BranchClose encountered.")
        if isinstance(node, ContextSymbol):
            self._context_stack[-1][type(node)] = node.model_copy()

    def get(self, cls: type[T_Context]) -> Optional[T_Context]:
        return self._context_stack[-1].get(cls)
