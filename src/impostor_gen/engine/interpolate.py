from abc import abstractmethod
from typing import Any, Optional, Tuple

from pydantic import BaseModel, Field

from .context import Context
from .ageing import Ageing
from .rule import Rule, Writer


class Interpolate(BaseModel):
    start_value: Optional[float] = None
    end_value: Optional[float] = None
    start_age: Optional[int] = None
    end_age: Optional[int] = None
    value_se: Optional[Tuple[float, float]] = Field(
        default=None, description="Interpolated value range (start, end)"
    )
    age_se: Optional[Tuple[int, int]] = Field(
        default=None, description="Age range (start, end)"
    )
    age_context_type: Optional[type[Ageing]] = Field(
        default=None, description="Context type to get age from"
    )

    @abstractmethod
    def set_interpolated_value(self, value: Any):
        pass


class InterpolateRule(Rule):
    def apply(self, writer: Writer, context: Context):
        interp = writer.peek(0)
        if not isinstance(interp, Interpolate):
            return
        # Check if all values are set
        start_value = interp.start_value or (
            interp.value_se[0] if interp.value_se else None
        )
        end_value = interp.end_value or (
            interp.value_se[1] if interp.value_se else None
        )
        start_age = interp.start_age or (interp.age_se[0] if interp.age_se else None)
        end_age = interp.end_age or (interp.age_se[1] if interp.age_se else None)
        age_context_type = interp.age_context_type

        # TODO warn for partial settings?

        if (
            start_value is None
            or end_value is None
            or start_age is None
            or end_age is None
            or age_context_type is None
        ):
            return

        age_context = context.get(age_context_type)
        if age_context is None:
            return

        age = age_context.age

        if age < start_age:
            value = start_value
        elif age > end_age:
            value = end_value
        else:
            t = (age - start_age) / (end_age - start_age)
            value = start_value + t * (end_value - start_value)

        interp.set_interpolated_value(value)
