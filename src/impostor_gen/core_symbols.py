
from impostor_gen.symbol import Symbol

from .interpolate import Interpolate


class Stem(Symbol):
    cross_sections: int = 3
    divisions: int = 5

    def __str__(self) -> str:
        return f"Stem({self.cross_sections}, {self.divisions})"


class F(Symbol, Interpolate):
    length: float = 1.0

    def __str__(self) -> str:
        return f"F({self.length:.2f})"
    
    def set_interpolated_value(self, value: float) -> None:
        self.length = value

class Diameter(Symbol, Interpolate):
    diameter: float = 1.0

    def __str__(self) -> str:
        return f"D({self.diameter:.2f})"
    
    def set_interpolated_value(self, value: float) -> None:
        self.diameter = value


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


class Tip(Symbol):
    order: int = 0 # Default to order 0 (main trunk)
    max_length: float = 10.0  # Maximum length before stopping growth

    def __str__(self) -> str:
        return "Tip"
    

class MaterialKey(Symbol):
    key: str

    def __str__(self) -> str:
        return f"MaterialKey({self.key})"