import numpy as np
from dataclasses import dataclass, replace
from scipy.interpolate import CubicSpline

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
    
    def integrate(self, a, b):
        """
        Integrate the curve between two parameter values.

        Args:
        a (float): The start parameter value.
        b (float): The end parameter value.

        Returns:
        float: The integral of the curve between a and b.
        """
        return self.curve.integrate(a, b)