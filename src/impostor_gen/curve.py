import numpy as np
from typing import Tuple, Optional

class BezierCurve2D:
    def __init__(
        self, 
        start_point: Tuple[float, float],
        control_point: Tuple[float, float],
        end_point: Tuple[float, float]
    ):
        """
        Initialize a 2D quadratic Bezier curve with x scaled from 0 to 1.
        
        Args:
            start_point: (x, y) coordinates of the start point
            control_point: (x, y) coordinates of the control point
            end_point: (x, y) coordinates of the end point
        """
        # Store original points
        self.original_start = np.array(start_point, dtype=float)
        self.original_control = np.array(control_point, dtype=float)
        self.original_end = np.array(end_point, dtype=float)
        
        # Scale x coordinates from 0 to 1
        x_min = min(start_point[0], control_point[0], end_point[0])
        x_max = max(start_point[0], control_point[0], end_point[0])
        x_range = max(x_max - x_min, 1e-10)  # Avoid division by zero
        
        # Scale x coordinates
        self.start_point = np.array([(start_point[0] - x_min) / x_range, start_point[1]])
        self.control_point = np.array([(control_point[0] - x_min) / x_range, control_point[1]])
        self.end_point = np.array([(end_point[0] - x_min) / x_range, end_point[1]])

    def point_at(self, t: float) -> np.ndarray:
        """Calculate a point on the Bezier curve at parameter t (0-1)."""
        return (
            (1 - t)**2 * self.start_point + 
            2 * (1 - t) * t * self.control_point + 
            t**2 * self.end_point
        )
    
    def y_at_x(self, x: float, samples: int = 100, tolerance: float = 1e-6) -> Optional[float]:
        """
        Find the y value at a given x position (0-1).
        
        Args:
            x: x-coordinate (0-1) to find the corresponding y value
            samples: number of samples to use for initial approximation
            tolerance: acceptable error in x for the final result
            
        Returns:
            The corresponding y value, or None if x is out of bounds (0-1)
        """
        if x < 0 or x > 1:
            return None
            
        # Sample the curve at regular intervals to find approximate t
        best_t = 0
        best_diff = float('inf')
        
        for i in range(samples):
            t = i / (samples - 1)
            point = self.point_at(t)
            diff = abs(point[0] - x)
            
            if diff < best_diff:
                best_diff = diff
                best_t = t
        
        # Refine the estimate using binary search
        t_low, t_high = max(0, best_t - 1/samples), min(1, best_t + 1/samples)
        
        while t_high - t_low > 1e-10 and best_diff > tolerance:
            t_mid = (t_low + t_high) / 2
            point = self.point_at(t_mid)
            diff = point[0] - x
            
            if abs(diff) < best_diff:
                best_diff = abs(diff)
                best_t = t_mid
                
            if diff > 0:
                t_high = t_mid
            else:
                t_low = t_mid
        
        # Return the y value at the best t
        return self.point_at(best_t)[1]