import numpy as np
from typing import List, Tuple, Optional, Union


class BezierCurve2D:
    def __init__(self, points: List[Tuple[float, float]]):
        """
        Initialize a piecewise 2D quadratic Bezier curve with x scaled from 0 to 1.

        Args:
            points: List of (x, y) coordinates where:
                - Even indices (0, 2, 4, ...) are endpoints
                - Odd indices (1, 3, 5, ...) are control points

        The curve consists of n segments, requiring 2n+1 points total:
        [endpoint0, control0, endpoint1, control1, endpoint2, ..., endpoint_n]
        """
        # Validate the number of points
        n = len(points)
        assert n >= 3 and n % 2 == 1, f"Expected 2n+1 points (n≥1), but got {n} points"

        # Store original points as numpy array
        self.original_points = np.array(points, dtype=float)

        # Check that x values of endpoints are strictly monotonic (increasing or decreasing)
        endpoints = self.original_points[::2]  # Even-indexed points
        x_diffs = np.diff(endpoints[:, 0])
        all_increasing = np.all(x_diffs > 0)
        all_decreasing = np.all(x_diffs < 0)

        assert all_increasing or all_decreasing, (
            "X values of endpoints must be consecutive (strictly monotonic)"
        )

        # Scale x coordinates from 0 to 1
        x_min = np.min(self.original_points[:, 0])
        x_max = np.max(self.original_points[:, 0])
        x_range = max(x_max - x_min, 1e-10)  # Avoid division by zero

        # Create scaled points
        self.points = self.original_points.copy()
        self.points[:, 0] = (self.points[:, 0] - x_min) / x_range

        # Determine number of segments
        self.n_segments = (n - 1) // 2

        # Separate endpoints and control points for easier access
        self.endpoints = self.points[::2]
        self.control_points = self.points[1::2]

        # Store if x values are increasing (for segment lookup)
        self.x_increasing = all_increasing

    def point_at(self, t: float) -> np.ndarray:
        """Calculate a point on the Bezier curve at parameter t (0-1)."""
        if t <= 0:
            return self.endpoints[0]
        elif t >= 1:
            return self.endpoints[-1]

        # Determine which segment t belongs to
        segment_idx = min(int(t * self.n_segments), self.n_segments - 1)

        # Scale t to local segment parameter [0-1]
        local_t = (t * self.n_segments) - segment_idx

        # Get points for this segment
        p0 = self.endpoints[segment_idx]
        p1 = self.control_points[segment_idx]
        p2 = self.endpoints[segment_idx + 1]

        # Quadratic Bezier formula
        return (
            (1 - local_t) ** 2 * p0 + 2 * (1 - local_t) * local_t * p1 + local_t**2 * p2
        )

    def _find_segment(self, x: float) -> int:
        """Find segment index containing the given x value."""
        if self.x_increasing:
            for i in range(self.n_segments):
                if self.endpoints[i][0] <= x <= self.endpoints[i + 1][0]:
                    return i
        else:
            for i in range(self.n_segments):
                if self.endpoints[i][0] >= x >= self.endpoints[i + 1][0]:
                    return i

        # Fallback to nearest endpoint segment
        if x < min(self.endpoints[0][0], self.endpoints[-1][0]):
            return 0
        return self.n_segments - 1

    def sample_y_evenly(self, num_points: int) -> List[float]:
        """Sample the curve at evenly spaced t values, including start and end."""
        t_values = np.linspace(0, 1, num_points)
        return [self.point_at(t)[1] for t in t_values]

    def plot(self, num_points: int = 100, figsize: Tuple[int, int] = (10, 6)):
        """
        Plot the Bezier curve and control points for visualization/debugging.

        Args:
            num_points: Number of points to sample along the curve
            figsize: Size of the figure (width, height) in inches
        """
        import matplotlib.pyplot as plt

        # Sample points along the curve
        t_values = np.linspace(0, 1, num_points)
        curve_points = np.array([self.point_at(t) for t in t_values])

        # Create plot
        plt.figure(figsize=figsize)

        # Plot the curve
        plt.plot(curve_points[:, 0], curve_points[:, 1], "b-", label="Bezier Curve")

        # Plot endpoints
        plt.plot(
            self.endpoints[:, 0],
            self.endpoints[:, 1],
            "ro",
            markersize=8,
            label="Endpoints",
        )

        # Plot control points
        plt.plot(
            self.control_points[:, 0],
            self.control_points[:, 1],
            "go",
            markersize=6,
            label="Control Points",
        )

        # Plot lines connecting control points to endpoints
        for i in range(self.n_segments):
            p0 = self.endpoints[i]
            p1 = self.control_points[i]
            p2 = self.endpoints[i + 1]
            plt.plot([p0[0], p1[0]], [p0[1], p1[1]], "g--", alpha=0.5)
            plt.plot([p1[0], p2[0]], [p1[1], p2[1]], "g--", alpha=0.5)

        plt.grid(True)
        plt.legend()
        plt.title("Bezier Curve Visualization")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    points: list[tuple[float, float]] = [(0, 0), (0.5, 1), (1, 0), (1.5, -1), (2, 0)]
    curve = BezierCurve2D(points)

    # Visualize the curve
    curve.plot()

    # Get y values for specific x values
    y = curve.y_at_x(0.5)
    y_values = curve.y_at_x([0.2, 0.4, 0.6, 0.8])
    print(f"y at x=0.5: {y}")
