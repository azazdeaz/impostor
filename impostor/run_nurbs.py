import numpy as np
import splinepy

# List of 3D points (example)
points = np.array([
    [0, 0, 0],
    [1, 1, 1],
    [2, 0, 2],
    [3, 1, 3],
    [4, 0, 4]
])

# Degree of the spline
degree = 3

# Number of control points
n_control_points = len(points)

# Generate a clamped knot vector
knot_vector = np.concatenate((
    [0] * (degree + 1),
    np.linspace(0, 1, n_control_points - degree + 1),
    [1] * (degree + 1)
))

# Create the spline curve
curve = splinepy.BezierSpline(
    degrees=[degree],
    control_points=points,
    knot_vectors=[knot_vector]
)

# Evaluate and plot the curve to see the effect of the knot vector
evaluated_points = curve.sample(100)
splinepy.plotting.plot(evaluated_points, show=True)
