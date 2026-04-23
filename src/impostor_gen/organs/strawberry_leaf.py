from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List

import numpy as np
import rerun as rr

from ..mesh.mesh3d import Mesh3D
from ..svg_guide import SvgGuide


def _spacing_starts_along_midrib(
    midrib_length_m: float,
    tip_clear_length_m: float,
    spacing_curve_m: np.ndarray,
    min_spacing_m: float,
) -> np.ndarray:
    """Return midrib arclength positions where secondary veins should start."""
    if midrib_length_m <= 0.0:
        return np.empty(0, dtype=np.float64)
    max_start_s = max(0.0, midrib_length_m - max(0.0, tip_clear_length_m))
    if max_start_s <= 0.0:
        return np.array([0.0], dtype=np.float64)

    n = len(spacing_curve_m)
    if n == 0:
        return np.empty(0, dtype=np.float64)

    spacing_curve_m = np.maximum(spacing_curve_m, min_spacing_m)
    s = 0.0
    starts: List[float] = [0.0]
    s_lookup = np.linspace(0.0, midrib_length_m, n, dtype=np.float64)

    while True:
        spacing = float(np.interp(s, s_lookup, spacing_curve_m))
        spacing = max(spacing, min_spacing_m)
        s += spacing
        if s >= max_start_s:
            break
        starts.append(s)

    return np.array(starts, dtype=np.float64)


def _build_secondary_vein_2d(
    start_xy: np.ndarray,
    length_m: float,
    start_angle: float,
    curvature_profile_per_cm: np.ndarray,
    side_sign: float,
    resolution_m: float,
) -> np.ndarray:
    """Integrate one 2D vein polyline with curvature sampled along progress."""
    if length_m <= 0.0:
        return np.array([start_xy], dtype=np.float64)

    
    points: List[np.ndarray] = [np.array(start_xy, dtype=np.float64)]
    theta = side_sign * start_angle
    remaining = float(length_m)
    traveled = 0.0
    curvature_u = np.linspace(0.0, 1.0, len(curvature_profile_per_cm))

    while remaining > 1e-9:
        ds = min(resolution_m, remaining)
        # Normalized progress t/u along this vein in [0, 1].
        u = min(traveled / length_m, 1.0)
        kappa_per_cm = side_sign * float(np.interp(u, curvature_u, curvature_profile_per_cm))
        theta += kappa_per_cm * (ds * 100.0)
        step = np.array([np.sin(theta), np.cos(theta)], dtype=np.float64) * ds
        points.append(points[-1] + step)
        remaining -= ds
        traveled += ds

    return np.array(points, dtype=np.float64)


def _centripetal_catmull_rom_2d(
    control_points: np.ndarray,
    samples_per_segment: int = 12,
) -> np.ndarray:
    """Sample a centripetal Catmull-Rom spline through 2D control points."""
    if len(control_points) <= 2:
        return control_points.copy()

    alpha = 0.5

    def tj(ti: float, pi: np.ndarray, pj: np.ndarray) -> float:
        return ti + max(float(np.linalg.norm(pj - pi)), 1e-9) ** alpha

    out: List[np.ndarray] = []
    n = len(control_points)
    for i in range(n - 1):
        p1 = control_points[i]
        p2 = control_points[i + 1]
        p0 = control_points[i - 1] if i > 0 else (p1 + (p1 - p2))
        p3 = control_points[i + 2] if i + 2 < n else (p2 + (p2 - p1))

        t0 = 0.0
        t1 = tj(t0, p0, p1)
        t2 = tj(t1, p1, p2)
        t3 = tj(t2, p2, p3)

        ts = np.linspace(t1, t2, samples_per_segment, endpoint=False)
        for t in ts:
            a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1
            a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
            a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
            b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
            b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3
            c = (t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2
            out.append(c)

    out.append(control_points[-1])
    return np.array(out, dtype=np.float64)


def _build_blade_curve_2d(midrib: np.ndarray, side_veins: List[np.ndarray]) -> np.ndarray:
    """Build one blade-side boundary through base, vein tips, and tip."""
    if len(midrib) == 0:
        return np.empty((0, 2), dtype=np.float64)
    base = midrib[0]
    tip = midrib[-1]
    vein_tips = [vein[-1] for vein in side_veins if len(vein) > 0]
    control = np.array([base, *vein_tips, tip], dtype=np.float64)
    return _centripetal_catmull_rom_2d(control)


def _resample_curve_with_spacing(
    curve: np.ndarray,
    spacing_curve_m: np.ndarray,
    min_spacing_m: float,
) -> np.ndarray:
    """Resample a polyline with variable spacing along arclength."""
    if len(curve) <= 2:
        return curve.copy()
    if len(spacing_curve_m) == 0:
        return curve.copy()

    segs = np.diff(curve, axis=0)
    seg_lens = np.linalg.norm(segs, axis=1)
    # Arclength coordinate (meters) for each original curve point.
    arc = np.concatenate(([0.0], np.cumsum(seg_lens)))
    total = float(arc[-1])
    if total <= 1e-9:
        return curve[:1].copy()

    spacing_curve_m = np.maximum(spacing_curve_m, min_spacing_m)
    # Lookup domain for spacing samples over arclength [0, total].
    s_lookup = np.linspace(0.0, total, len(spacing_curve_m), dtype=np.float64)

    # Target arclength positions (meters) for resampled points.
    samples = [0.0]
    s = 0.0
    last_spacing = min_spacing_m
    while True:
        spacing = float(np.interp(s, s_lookup, spacing_curve_m))
        last_spacing = max(spacing, min_spacing_m)
        s_next = s + last_spacing
        if s_next >= total:
            break
        samples.append(s_next)
        s = s_next

    # Avoid a tiny final segment:
    # - if tail is short (< 0.5 spacing), stretch existing samples
    # - otherwise add one full spacing step, then scale back to [0, total].
    tail = total - samples[-1]
    if len(samples) == 1:
        samples.append(total)
    else:
        if tail >= 0.5 * last_spacing:
            samples.append(samples[-1] + last_spacing)
        end_before_scale = samples[-1]
        if end_before_scale > 1e-9:
            scale = total / end_before_scale
            samples = [v * scale for v in samples]

    samples_s = np.array(samples, dtype=np.float64)

    # np.interp expects strictly increasing xp; keep first of duplicated arc entries.
    unique_idx = np.unique(arc, return_index=True)[1]
    arc_u = arc[np.sort(unique_idx)]
    x_u = curve[np.sort(unique_idx), 0]
    y_u = curve[np.sort(unique_idx), 1]
    x = np.interp(samples_s, arc_u, x_u)
    y = np.interp(samples_s, arc_u, y_u)
    return np.column_stack((x, y))


def _inflate_curve_along_normals(
    curve: np.ndarray,
    inflation_curve_m: np.ndarray,
    side_sign: float,
) -> np.ndarray:
    """Push curve points outward along normals by guide-controlled offsets."""
    if len(curve) <= 2 or len(inflation_curve_m) == 0:
        return curve.copy()

    # u_curve/u_offset are normalized [0, 1] positions along blade arclength.
    u_curve = np.linspace(0.0, 1.0, len(curve), dtype=np.float64)
    u_offset = np.linspace(0.0, 1.0, len(inflation_curve_m), dtype=np.float64)
    offsets = np.interp(u_curve, u_offset, inflation_curve_m)

    out = curve.copy()
    for i in range(len(curve)):
        if i == 0:
            # The first point is the midrib base, so the normal should point down.
            # TODO: This should point in the opposite direction of the first section of the midrib.
            normal = np.array([0.0, -1.0], dtype=np.float64)
        elif i == len(curve) - 1:
            # The last point is the midrib tip. (Same as the first point, but in the opposite direction.)
            normal = np.array([0.0, 1.0], dtype=np.float64)
        else:
            tangent = curve[i + 1] - curve[i - 1]
            tnorm = float(np.linalg.norm(tangent))
            if tnorm <= 1e-9:
                continue
            tangent /= tnorm
            normal = np.array([-tangent[1], tangent[0]], dtype=np.float64)
            if normal[0] * side_sign < 0.0:
                normal = -normal

        out[i] = curve[i] + normal * offsets[i]

    return out


@dataclass
class StrawberryLeaf:
    midrib_positions_2d: np.ndarray
    midrib_length_m: float
    vein_resolution_m: float
    vein_free_tip_length_m: float
    left_secondary_veins_2d: List[np.ndarray]
    right_secondary_veins_2d: List[np.ndarray]
    left_blade_curve_2d: np.ndarray
    right_blade_curve_2d: np.ndarray

    @classmethod
    def from_guide(
        cls,
        guide_path: str | Path,
    ) -> "StrawberryLeaf":
        guide = SvgGuide(guide_path)
        midrib_length_m = guide.get_meter("midrib_length")
        vein_resolution_m = guide.get_meter("vein_resolution")
        vein_free_tip_length_m = guide.get_meter("vein_free_tip_length")
        min_spacing_m = max(vein_resolution_m, 1e-6)

        spacing_curve_m = np.array(
            guide.get_meters("leaflet.secondary_veins.vein_spacing", 128),
            dtype=np.float64,
        )
        blade_point_spacing_m = np.array(
            guide.get_meters("leaflet.secondary_veins.blade_point_spacing", 128),
            dtype=np.float64,
        )
        blade_inflation_m = np.array(
            guide.get_meters("leaflet.secondary_veins.blade_inflation", 128),
            dtype=np.float64,
        )
        vein_start_s = _spacing_starts_along_midrib(
            midrib_length_m=midrib_length_m,
            tip_clear_length_m=vein_free_tip_length_m,
            spacing_curve_m=spacing_curve_m,
            min_spacing_m=min_spacing_m,
        )

        # Starts at (0, 0) and grows toward +Y.
        y = np.concatenate((vein_start_s, [midrib_length_m]))
        midrib_positions_2d = np.column_stack((np.zeros_like(y), y))

        n_veins = len(vein_start_s)
        lengths_m = np.array(
            guide.get_meters("leaflet.secondary_veins.lengths", n_veins),
            dtype=np.float64,
        )
        start_angles = np.array(
            guide.get_radians("leaflet.secondary_veins.start_angles", n_veins),
            dtype=np.float64,
        )
        max_steps_per_vein = max(2, int(np.ceil(lengths_m.max() / vein_resolution_m))) if n_veins else 2
        curvature_profile_per_cm = np.array(
            guide.get_radians("leaflet.secondary_veins.curvature", max_steps_per_vein),
            dtype=np.float64,
        )

        left_secondary_veins_2d: List[np.ndarray] = []
        right_secondary_veins_2d: List[np.ndarray] = []
        for i, s in enumerate(vein_start_s):
            start_xy = np.array([0.0, s], dtype=np.float64)
            left = _build_secondary_vein_2d(
                start_xy=start_xy,
                length_m=float(lengths_m[i]),
                start_angle=float(start_angles[i]),
                curvature_profile_per_cm=curvature_profile_per_cm,
                side_sign=-1.0,
                resolution_m=vein_resolution_m,
            )
            right = _build_secondary_vein_2d(
                start_xy=start_xy,
                length_m=float(lengths_m[i]),
                start_angle=float(start_angles[i]),
                curvature_profile_per_cm=curvature_profile_per_cm,
                side_sign=1.0,
                resolution_m=vein_resolution_m,
            )
            left_secondary_veins_2d.append(left)
            right_secondary_veins_2d.append(right)

        left_blade_curve_2d = _build_blade_curve_2d(midrib_positions_2d, left_secondary_veins_2d)
        right_blade_curve_2d = _build_blade_curve_2d(midrib_positions_2d, right_secondary_veins_2d)
        left_blade_curve_2d = _resample_curve_with_spacing(
            left_blade_curve_2d, blade_point_spacing_m, min_spacing_m
        )
        right_blade_curve_2d = _resample_curve_with_spacing(
            right_blade_curve_2d, blade_point_spacing_m, min_spacing_m
        )
        left_blade_curve_2d = _inflate_curve_along_normals(
            left_blade_curve_2d, blade_inflation_m, side_sign=-1.0
        )
        right_blade_curve_2d = _inflate_curve_along_normals(
            right_blade_curve_2d, blade_inflation_m, side_sign=1.0
        )

        return cls(
            midrib_positions_2d=midrib_positions_2d,
            midrib_length_m=midrib_length_m,
            vein_resolution_m=vein_resolution_m,
            vein_free_tip_length_m=vein_free_tip_length_m,
            left_secondary_veins_2d=left_secondary_veins_2d,
            right_secondary_veins_2d=right_secondary_veins_2d,
            left_blade_curve_2d=left_blade_curve_2d,
            right_blade_curve_2d=right_blade_curve_2d,
        )

    def get_visual_mesh(self) -> Mesh3D:
        return Mesh3D.empty()

    def get_collision_mesh(self) -> Mesh3D:
        return Mesh3D.empty()

    def log_structure(self, path: str = "strawberry_leaf/structure") -> None:
        strips_3d: List[np.ndarray] = []

        midrib_3d = np.column_stack(
            (
                self.midrib_positions_2d[:, 0],
                self.midrib_positions_2d[:, 1],
                np.zeros(len(self.midrib_positions_2d), dtype=np.float64),
            )
        )
        strips_3d.append(midrib_3d)

        for vein in self.left_secondary_veins_2d + self.right_secondary_veins_2d:
            vein_3d = np.column_stack(
                (
                    vein[:, 0],
                    vein[:, 1],
                    np.zeros(len(vein), dtype=np.float64),
                )
            )
            strips_3d.append(vein_3d)

        rr.log(path, rr.LineStrips3D(strips_3d))

        left_blade_3d = np.column_stack(
            (
                self.left_blade_curve_2d[:, 0],
                self.left_blade_curve_2d[:, 1],
                np.zeros(len(self.left_blade_curve_2d), dtype=np.float64),
            )
        )
        right_blade_3d = np.column_stack(
            (
                self.right_blade_curve_2d[:, 0],
                self.right_blade_curve_2d[:, 1],
                np.zeros(len(self.right_blade_curve_2d), dtype=np.float64),
            )
        )
        blade_points = np.vstack((left_blade_3d, right_blade_3d))
        rr.log(f"{path}/blades", rr.Points3D(blade_points))