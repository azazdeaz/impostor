from pydantic import BaseModel, Field

from .context import Context, ContextSymbol
from .rule import Rule, Writer


class Ageing(ContextSymbol):
    age: float = Field(default=0.0, description="Current age of the context")


class AgeingRule(Rule, BaseModel):
    aging_per_iteration: float = Field(
        default=1.0, description="Aging increment per iteration"
    )

    def apply(self, writer: Writer, context: Context):
        aging = writer.peek(0)
        if not isinstance(aging, Ageing):
            return

        aging.age += self.aging_per_iteration
