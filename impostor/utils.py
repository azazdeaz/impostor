import numpy as np
from dataclasses import dataclass, replace
from scipy.interpolate import CubicSpline

@dataclass
class NormalDistribution:
    """
    Normal distribution based random number generator that can be configured with a mean and standard deviation.

    Attributes:
    mean (float): The mean of the normal distribution.
    std (float): The standard deviation of the normal distribution.
    """
    mean: float
    std: float

    def sample(self):
        # TODO: Add option to seed the random number generator.
        return np.random.normal(self.mean, self.std)
    

def fluent_methods(cls):
    for field in cls.__dataclass_fields__:
        setattr(
            cls,
            f"with_{field}",
            lambda self, val, field=field: replace(self, **{field: val}),
        )
    return cls

class Curve:
    def __init__(self, control_points: list[tuple[float, float]]):
        """
        Initialize the curve with control points.

        Args:
        control_points (list[tuple[float, float]]): The control points of the curve.
        """
        self._control_points = control_points
        self.curve = CubicSpline(*zip(*self._control_points), bc_type="not-a-knot")
        

    def evaluate(self, t):
        """
        Evaluate the curve at t.

        Args:
        t (float): The parameter value at which to evaluate the curve.

        Returns:
        tuple[float, float]: The point on the curve at t.
        """
        return self.curve(t)