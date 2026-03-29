from __future__ import annotations

from typing import Any, List

import numpy as np
import triangle as tr
from scipy.interpolate import make_interp_spline

from .mesh_context import MeshContext
from .mesh3d import Mesh3D


# ── Collider mesh (unchanged layer-strip approach) ────────────────────


def _triangulate_leaf(blueprint: Any) -> Mesh3D:
    """Build a triangulated Mesh3D from a LeafBlueprint's midrib + veins."""
    midrib_div = len(blueprint.midrib.transforms)

    layers: List[List[np.ndarray]] = []

    assert len(blueprint.veins) == (midrib_div - 1) * 2, (
        f"Each midrib section must have two secondary veins except the tip. "
        f"Found {len(blueprint.veins)} veins for {midrib_div} midrib sections."
    )

    for i in range(midrib_div):
        if i == midrib_div - 1:
            layers.append([blueprint.midrib.transforms[i].position])
        else:
            layers.append(
                [t.position for t in blueprint.veins[i * 2].transforms[::-1]]
                + [blueprint.midrib.transforms[i].position]
                + [t.position for t in blueprint.veins[i * 2 + 1].transforms]
            )

    vertices = np.concatenate(layers)
    one_start = 0
    faces: List[List[int]] = []
    is_closed = False
    winding_clockwise = True

    for i in range(len(layers) - 1):
        two_start = one_start + len(layers[i])
        one_id = 0
        two_id = 0
        one_size = len(layers[i])
        two_size = len(layers[i + 1])

        id_margin = 0 if is_closed else 1
        while one_id < one_size - id_margin or two_id < two_size - id_margin:
            id1 = one_start + one_id % one_size
            id2 = two_start + two_id % two_size
            id1_next = one_start + (one_id + 1) % one_size
            id2_next = two_start + (two_id + 1) % two_size

            up_progress = one_id / one_size
            down_progress = two_id / two_size

            if up_progress > down_progress:
                if winding_clockwise:
                    new_face = [id1, id2_next, id2]
                else:
                    new_face = [id1, id2, id2_next]
                two_id += 1
            else:
                if winding_clockwise:
                    new_face = [id1, id1_next, id2]
                else:
                    new_face = [id1, id2, id1_next]
                one_id += 1

            faces.append(new_face)

        one_start = two_start

    return Mesh3D(
        vertex_positions=vertices,
        triangle_indices=np.array(faces),
        material_key=blueprint.material_key,
    )


# ── Serrated visual mesh ─────────────────────────────────────────────


def _fit_spline(points: np.ndarray):
    """Cubic interpolating spline through 3D points.  Returns callable(t)->3D."""
    t = np.linspace(0, 1, len(points))
    k = min(3, len(points) - 1)
    return make_interp_spline(t, points, k=k)


def _sample_peaks(spline, n: int, rng: np.random.Generator) -> np.ndarray:
    """Sample *n* points evenly spaced by arc length, with slight jitter."""
    # Dense sampling to approximate arc length
    t_dense = np.linspace(0, 1, 500)
    pts_dense = np.array([spline(ti) for ti in t_dense])
    seg_lens = np.linalg.norm(np.diff(pts_dense, axis=0), axis=1)
    arc = np.concatenate([[0], np.cumsum(seg_lens)])
    total = arc[-1]

    # Even arc-length targets (skip endpoints = base/tip)
    s_targets = np.linspace(0, total, n + 2)[1:-1]
    jitter = rng.uniform(-0.3 * total / n, 0.3 * total / n, n)
    s_targets = np.clip(s_targets + jitter, total * 0.01, total * 0.99)

    # Map arc lengths back to parameter t
    t_vals = np.interp(s_targets, arc, t_dense)
    return np.array([spline(ti) for ti in t_vals])


def _collect_edge_guides(blueprint: Any) -> tuple[np.ndarray, np.ndarray]:
    """For each vein, return (tip_position, inward_direction) arrays.

    The inward direction is the vector from the vein tip one step toward
    the midrib (transforms[-1] → transforms[-2])."""
    positions, directions = [], []
    for vein in blueprint.veins:
        tip = vein.transforms[-1].position
        inward = vein.transforms[-2].position
        positions.append(tip)
        directions.append(inward - tip)
    return np.array(positions), np.array(directions)


def _interleave_valleys(
    peaks: np.ndarray,
    guide_positions: np.ndarray,
    guide_directions: np.ndarray,
    depth: float,
) -> np.ndarray:
    """Return peak→valley→peak→… array.

    Valleys sit 1/3 of the way from the first peak toward the next,
    then pushed inward along the closest vein's inward direction.
    *depth* scales the push relative to that vein segment's length."""
    out: list[np.ndarray] = []
    for i, pk in enumerate(peaks):
        out.append(pk)
        if i < len(peaks) - 1:
            pt = peaks[i] * (2 / 3) + peaks[i + 1] * (1 / 3)
            dists = np.linalg.norm(guide_positions - pt, axis=1)
            closest = np.argmin(dists)
            direction = guide_directions[closest]
            pt = pt + direction * depth
            out.append(pt)
    return np.array(out)


def _project_to_2d(pts: np.ndarray):
    """PCA-project 3D points onto the leaf's dominant plane.
    Returns (pts_2d, centroid, basis[2×3])."""
    center = pts.mean(axis=0)
    centered = pts - center
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    return centered @ vt[:2].T, center, vt[:2]


def _build_serrated_visual(
    blueprint: Any,
    teeth_per_side: int = 20,
    serration_depth: float = 0.3,
) -> Mesh3D:
    """Build a leaf mesh with serrated (saw-tooth) edges.

    1. Fit two splines through the vein tips (left / right halves).
    2. Sample peaks on each spline; insert valley points between them.
    3. Constrained-Delaunay triangulate the serrated boundary + interior
       midrib/vein vertices projected onto the leaf plane.
    """
    midrib_div = len(blueprint.midrib.transforms)
    midrib_pts = np.array([t.position for t in blueprint.midrib.transforms])
    base, tip = midrib_pts[0], midrib_pts[-1]

    # ── 1. Vein tips as spline control points ──
    n_sections = midrib_div - 1
    left_tips = [blueprint.veins[i * 2].transforms[-1].position
                 for i in range(n_sections)]
    right_tips = [blueprint.veins[i * 2 + 1].transforms[-1].position
                  for i in range(n_sections)]

    # Two independent splines → hard break at tip, meet at base
    left_spline = _fit_spline(np.array([base, *left_tips, tip]))
    right_spline = _fit_spline(np.array([tip, *right_tips[::-1], base]))

    # ── 2. Peaks + valleys → serrated edge ──
    guide_pos, guide_dir = _collect_edge_guides(blueprint)
    rng = np.random.default_rng(42)
    left_edge = _interleave_valleys(
        _sample_peaks(left_spline, teeth_per_side, rng),
        guide_pos, guide_dir, serration_depth,
    )
    right_edge = _interleave_valleys(
        _sample_peaks(right_spline, teeth_per_side, rng),
        guide_pos, guide_dir, serration_depth,
    )
    boundary = np.array([base, *left_edge, tip, *right_edge])

    # ── 3. Interior points (midrib + vein interiors, no duplicates) ──
    interior_list: list[np.ndarray] = list(midrib_pts[1:-1])
    for vein in blueprint.veins:
        interior_list.extend(t.position for t in vein.transforms[1:-1])
    interior = np.array(interior_list) if interior_list else np.empty((0, 3))

    # ── 4. Project → constrained Delaunay → reconstruct ──
    all_3d = np.vstack([boundary, interior]) if len(interior) else boundary
    pts_2d, center, basis = _project_to_2d(all_3d)

    n_bnd = len(boundary)
    segments = np.array([[i, (i + 1) % n_bnd] for i in range(n_bnd)])
    hole = pts_2d.min(axis=0) - 1.0       # a point guaranteed outside the leaf

    tri_out = tr.triangulate(
        {"vertices": pts_2d, "segments": segments, "holes": [hole.tolist()]}, "p",
    )

    # If the triangulator added Steiner points, unproject them to the leaf plane
    out_verts_2d = tri_out["vertices"]
    if len(out_verts_2d) > len(all_3d):
        extra_3d = out_verts_2d[len(all_3d):] @ basis + center
        all_3d = np.vstack([all_3d, extra_3d])

    return Mesh3D(
        vertex_positions=all_3d,
        triangle_indices=tri_out["triangles"],
        material_key=blueprint.material_key,
    )


class LeafMeshContext(MeshContext):
    """Generates triangulated leaf meshes from midrib + vein blueprints."""

    def generate_visual(self, blueprint: Any) -> Mesh3D:
        # return _triangulate_leaf(blueprint)
        return _build_serrated_visual(blueprint)

    def generate_collider(self, blueprint: Any) -> Mesh3D:
        return _triangulate_leaf(blueprint)
