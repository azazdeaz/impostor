from abc import abstractmethod
from typing import Any, Optional

from pydantic import BaseModel

from impostor_gen.context import Context, LeafContext
from impostor_gen.rule import Rule, Writer


class Interpolate(BaseModel):
    start_value: Optional[float] = None
    end_value: Optional[float] = None
    start_age: Optional[int] = None
    end_age: Optional[int] = None
    age_context_type: Optional[type[LeafContext]] = None

    @abstractmethod
    def set_interpolated_value(self, value: Any):
        pass


class InterpolateRule(Rule):
    def apply(self, writer: Writer, context: Context):
        interp = writer.peek(0)
        if not isinstance(interp, Interpolate):
            return
        # Check if all values are set
        if (
            interp.start_value is None
            or interp.end_value is None
            or interp.start_age is None
            or interp.end_age is None
            or interp.age_context_type is None
        ):
            return

        age_context = context.get(interp.age_context_type)
        if age_context is None:
            return

        age = age_context.age

        if age < interp.start_age:
            value = interp.start_value
        elif age > interp.end_age:
            value = interp.end_value
        else:
            t = (age - interp.start_age) / (interp.end_age - interp.start_age)
            value = interp.start_value + t * (interp.end_value - interp.start_value)

        interp.set_interpolated_value(value)
