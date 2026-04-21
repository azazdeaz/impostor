"""Load curve parameters from Inkscape SVG files.

Curves are identified by dotted label paths matching the SVG layer/group
hierarchy (e.g. ``"leaflet.secondary_veins.lengths"``).  A sibling path
labeled ``"ZERO"`` in the same group defines y = 0.
"""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_SVG_NS = "http://www.w3.org/2000/svg"
_INKSCAPE_NS = "http://www.inkscape.org/namespaces/inkscape"

# ── SVG path `d` tokenizer / parser ──────────────────────────────────

_TOKEN_RE = re.compile(r"[MmCcLlHhVvSsZz]|[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?")


def _parse_d(d: str) -> list[tuple[float, float, float, float, float, float, float, float]]:
    """Parse an SVG path ``d`` attribute into a list of absolute cubic Bézier segments.

    Each segment is ``(x0, y0, cx1, cy1, cx2, cy2, x1, y1)``.
    Handles M/m, L/l, H/h, V/v, C/c, S/s, Z/z.
    """
    tokens = _TOKEN_RE.findall(d)
    idx = 0
    cx = cy = 0.0  # current point
    sx = sy = 0.0  # subpath start
    segments: list[tuple[float, ...]] = []
    prev_cmd = ""
    prev_cx2 = prev_cy2 = 0.0  # last control point for S/s

    def _next() -> float:
        nonlocal idx
        val = float(tokens[idx])
        idx += 1
        return val

    while idx < len(tokens):
        tok = tokens[idx]
        if tok.isalpha():
            cmd = tok
            idx += 1
        else:
            # Implicit repeat of previous command (L after M)
            cmd = prev_cmd if prev_cmd else "L"

        if cmd in ("M", "m"):
            x, y = _next(), _next()
            if cmd == "m":
                x += cx; y += cy
            cx, cy = x, y
            sx, sy = x, y
            prev_cmd = "L" if cmd == "M" else "l"
            continue

        if cmd in ("L", "l"):
            x, y = _next(), _next()
            if cmd == "l":
                x += cx; y += cy
            segments.append((cx, cy, cx, cy, x, y, x, y))
            cx, cy = x, y

        elif cmd in ("H", "h"):
            x = _next()
            if cmd == "h":
                x += cx
            segments.append((cx, cy, cx, cy, x, cy, x, cy))
            cx = x

        elif cmd in ("V", "v"):
            y = _next()
            if cmd == "v":
                y += cy
            segments.append((cx, cy, cx, cy, cx, y, cx, y))
            cy = y

        elif cmd in ("C", "c"):
            cx1, cy1, cx2, cy2, x, y = _next(), _next(), _next(), _next(), _next(), _next()
            if cmd == "c":
                cx1 += cx; cy1 += cy; cx2 += cx; cy2 += cy; x += cx; y += cy
            segments.append((cx, cy, cx1, cy1, cx2, cy2, x, y))
            prev_cx2, prev_cy2 = cx2, cy2
            cx, cy = x, y

        elif cmd in ("S", "s"):
            cx1 = 2 * cx - prev_cx2
            cy1 = 2 * cy - prev_cy2
            cx2, cy2, x, y = _next(), _next(), _next(), _next()
            if cmd == "s":
                cx2 += cx; cy2 += cy; x += cx; y += cy
            segments.append((cx, cy, cx1, cy1, cx2, cy2, x, y))
            prev_cx2, prev_cy2 = cx2, cy2
            cx, cy = x, y

        elif cmd in ("Z", "z"):
            if (cx, cy) != (sx, sy):
                segments.append((cx, cy, cx, cy, sx, sy, sx, sy))
            cx, cy = sx, sy

        prev_cmd = cmd

    return segments  # type: ignore[return-value]


def _sample_cubic(seg: tuple[float, ...], n: int) -> np.ndarray:
    """Sample *n* points along a single cubic Bézier segment."""
    x0, y0, cx1, cy1, cx2, cy2, x1, y1 = seg
    t = np.linspace(0, 1, n).reshape(-1, 1)
    p0 = np.array([x0, y0])
    p1 = np.array([cx1, cy1])
    p2 = np.array([cx2, cy2])
    p3 = np.array([x1, y1])
    return (1 - t) ** 3 * p0 + 3 * (1 - t) ** 2 * t * p1 + 3 * (1 - t) * t ** 2 * p2 + t ** 3 * p3


# ── Parsed curve data ────────────────────────────────────────────────

@dataclass
class _CurveData:
    segments: list[tuple[float, float, float, float, float, float, float, float]]
    zero_y: float

    def sample(self, num_points: int) -> list[float]:
        """Sample the curve at *num_points* positions evenly spaced in x.

        Returns y-displacements from the ZERO line (above ZERO = positive).
        """
        # Dense sampling across all segments
        pts_per_seg = max(200, 500 // len(self.segments))
        all_pts = np.vstack([_sample_cubic(s, pts_per_seg) for s in self.segments])
        xs, ys = all_pts[:, 0], all_pts[:, 1]

        # Ensure monotonic x for interpolation
        order = np.argsort(xs)
        xs, ys = xs[order], ys[order]

        # Sample at evenly spaced x positions (normalized 0→1)
        x_norm = np.linspace(xs[0], xs[-1], num_points)
        y_interp = np.interp(x_norm, xs, ys)
        return (self.zero_y - y_interp).tolist()


# ── Public API ───────────────────────────────────────────────────────

class SvgGuide:
    """Load curve parameters from an Inkscape SVG file.

    Curves are addressed by dotted label paths matching the SVG
    ``inkscape:label`` hierarchy, e.g. ``"leaflet.secondary_veins.lengths"``.

    A sibling ``<path>`` labeled ``"ZERO"`` in each group defines y = 0.
    """

    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)
        self._curves: dict[str, _CurveData] | None = None

    def _ensure_parsed(self) -> dict[str, _CurveData]:
        if self._curves is None:
            self._curves = _parse_svg(self._path)
        return self._curves

    def _resolve_curve(self, dotted_path: str) -> _CurveData:
        curves = self._ensure_parsed()
        if dotted_path in curves:
            return curves[dotted_path]

        # Robust fallback: allow looking up by leaf label if hierarchy changed.
        suffix = f".{dotted_path}"
        matches = [curve for key, curve in curves.items() if key.endswith(suffix)]
        if len(matches) == 1:
            return matches[0]

        raise KeyError(dotted_path)

    def get_mm(self, dotted_path: str, num_points: int) -> list[float]:
        """Sample curve as mm displacements from the ZERO line."""
        return self._resolve_curve(dotted_path).sample(num_points)

    def get_meters(self, dotted_path: str, num_points: int) -> list[float]:
        """Sample curve as meters (1 mm in SVG = 0.001 m)."""
        return [v / 1000.0 for v in self.get_mm(dotted_path, num_points)]

    def get_meter(self, dotted_path: str) -> float:
        """Return path X-extent in meters (max_x - min_x)."""
        segments = self._resolve_curve(dotted_path).segments
        xs = np.array(
            [x for (x0, _, cx1, _, cx2, _, x1, _) in segments for x in (x0, cx1, cx2, x1)],
            dtype=np.float64,
        )
        if xs.size == 0:
            return 0.0
        return float(xs.max() - xs.min()) / 1000.0

    def get_radians(self, dotted_path: str, num_points: int) -> list[float]:
        """Sample curve as radians (1 cm in SVG = 1 radian)."""
        return [v / 10.0 for v in self.get_mm(dotted_path, num_points)]

    def get_degrees(self, dotted_path: str, num_points: int) -> list[float]:
        """Sample curve as degrees (1 cm in SVG = 1 radian, converted)."""
        return [math.degrees(v) for v in self.get_radians(dotted_path, num_points)]


# ── SVG tree walker ──────────────────────────────────────────────────

def _find_zero_y(group: ET.Element) -> float:
    """Find the y-coordinate of the ZERO path in a group."""
    for el in group:
        if el.tag == f"{{{_SVG_NS}}}path" and el.get(f"{{{_INKSCAPE_NS}}}label") == "ZERO":
            d = el.get("d", "")
            segs = _parse_d(d)
            if segs:
                return segs[0][1]  # y0 of first segment
    return 0.0


def _parse_svg(path: Path) -> dict[str, _CurveData]:
    tree = ET.parse(path)
    curves: dict[str, _CurveData] = {}

    def walk(el: ET.Element, prefix: str, zero_y: float) -> None:
        label = el.get(f"{{{_INKSCAPE_NS}}}label")
        is_group = el.tag == f"{{{_SVG_NS}}}g"

        if is_group:
            key = f"{prefix}.{label}" if prefix and label else (label or prefix)
            local_zero = _find_zero_y(el)
            if local_zero != 0.0:
                zero_y = local_zero
            for child in el:
                walk(child, key, zero_y)

        elif el.tag == f"{{{_SVG_NS}}}path" and label and label != "ZERO":
            key = f"{prefix}.{label}" if prefix else label
            d = el.get(f"{{{_INKSCAPE_NS}}}original-d") or el.get("d", "")
            segs = _parse_d(d)
            if segs:
                curves[key] = _CurveData(segments=segs, zero_y=zero_y)

    for child in tree.getroot():
        walk(child, "", 0.0)

    return curves
