from typing import Optional, Tuple
from pydantic import Field

from .context import Context
from .rule import Rule, Writer
from .ageing import AgeingContext
from .core_symbols import F, Diameter, Pitch, Symbol

import numpy as np

class StemTip(Symbol):
    """Marker symbol for the tip of the stem."""
    num_sections: int = 0

    def __str__(self) -> str:
        return "StemTip()"


class StemContext(AgeingContext):
    target_length: float = Field(default=1.0, description="Target length of the stem")
    growth_speed: float = Field(default=0.2, description="Growth speed per age unit")
    section_length: float = Field(
        default=0.2, description="Length of each stem section segment"
    )

    def section_growth_time(self) -> float:
        return self.section_length / self.growth_speed

    def section_growth_schedule(self, section_index: int) -> Tuple[float, float]:
        """Returns the start and end age for the growth of the given section index."""
        start_age = section_index * self.section_growth_time()
        end_age = start_age + self.section_growth_time()
        return (start_age, end_age)
    
    def create_next_section(self, created_section_count:int) -> Optional[F]:
        target_length = np.minimum(self.target_length, 
                                  self.growth_speed * self.age)
        target_section_count = int(np.ceil(target_length / self.section_length))
        if target_section_count > created_section_count:
            return F(
                value_se=(0.0, self.section_length),
                age_se=self.section_growth_schedule(created_section_count),
                age_context_type=StemContext,
            )


class StemGrowthRule(Rule):
    def apply(self, writer: Writer, context: Context):
        tip = writer.peek(0)
        if not isinstance(tip, StemTip):
            return

        stem_context = context.get(StemContext)
        if stem_context is None:
            return


        # Extend the stem
        next_section = stem_context.create_next_section(tip.num_sections)
        if next_section is not None:
            writer.write(
                [
                    Pitch(angle=-2),
                    Diameter(diameter=0.2),
                    next_section,
                    StemTip(num_sections=tip.num_sections + 1),
                ]
            )
