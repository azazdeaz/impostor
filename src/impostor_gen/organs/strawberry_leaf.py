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
        u = min(traveled / length_m, 1.0)
        kappa_per_cm = side_sign * float(np.interp(u, curvature_u, curvature_profile_per_cm))
        theta += kappa_per_cm * (ds * 100.0)
        step = np.array([np.sin(theta), np.cos(theta)], dtype=np.float64) * ds
        points.append(points[-1] + step)
        remaining -= ds
        traveled += ds

    return np.array(points, dtype=np.float64)


@dataclass
class StrawberryLeaf:
    midrib_positions_2d: np.ndarray
    midrib_length_m: float
    vein_resolution_m: float
    vein_free_tip_length_m: float
    secondary_veins_2d: List[np.ndarray]

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

        secondary_veins_2d: List[np.ndarray] = []
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
            secondary_veins_2d.extend([left, right])

        return cls(
            midrib_positions_2d=midrib_positions_2d,
            midrib_length_m=midrib_length_m,
            vein_resolution_m=vein_resolution_m,
            vein_free_tip_length_m=vein_free_tip_length_m,
            secondary_veins_2d=secondary_veins_2d,
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

        for vein in self.secondary_veins_2d:
            vein_3d = np.column_stack(
                (
                    vein[:, 0],
                    vein[:, 1],
                    np.zeros(len(vein), dtype=np.float64),
                )
            )
            strips_3d.append(vein_3d)

        rr.log(path, rr.LineStrips3D(strips_3d))