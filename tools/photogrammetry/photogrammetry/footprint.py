"""Camera model + per-frame ground footprint + coverage union.

Per DES-005 D3/D4, `--mode check` predicts survey coverage by projecting
each frame's image-corner rays onto the ground plane using the manifest
camera intrinsics and the recorded pose, then unioning the per-frame
footprints. This module is pure numpy + shapely -- no image data is read,
no reconstruction is performed (see run_pipeline.py's check-mode
invariant: no ODM, no SfM/MVS).

# Frame convention (T3.2 design note)
DES-004 records only the vehicle body pose (MAVROS `local_position/pose`,
x/y/z + qx/qy/qz/qw) -- no separate camera extrinsic/lever-arm/gimbal
calibration is captured. DES-003 fixes the survey polygon and the pose's
x/y as the *same* local ENU metric frame ("vertices in local ENU frame, z
ignored"), so footprints and the manifest polygon can be intersected
directly with no additional georeferencing step. Given the absence of a
recorded camera extrinsic, this module treats the camera as rigidly
mounted with no position offset from the recorded pose (camera position ==
pose position) and fixes the following boresight convention:

  - In the *unrotated* local axes, the camera boresight (principal ray,
    increasing depth) points straight down: (0, 0, -1). This makes an
    identity pose quaternion (qx=qy=qz=0, qw=1) a nadir-pointing camera --
    the v1 fixed downward-facing survey camera DES-003/004 assume, with no
    gimbal modeled. The recorded pose quaternion is applied as the
    camera-to-world rotation on top of this fixed downward boresight.
  - Pixel (u, v) -> local (pre-rotation) ray direction:
    ((u - cx) / fx, -(v - cy) / fy, -1) -- standard pinhole projection,
    with image row v flipped so "up in the image" is "up" in the
    pre-rotation local frame.

If a future task adds a recorded camera-to-body extrinsic to DES-004, it
composes with the fixed boresight rotation used here; until then this is
the documented, fixed convention for both check and full mode footprint
math (coverage.py is engine-agnostic and does not care how the covered
region was produced).
"""
from __future__ import annotations

import dataclasses
import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np
from shapely.geometry import MultiPoint, Polygon
from shapely.ops import unary_union

from photogrammetry.dataset import PoseRow

REQUIRED_INTRINSICS_KEYS = ("fx", "fy", "cx", "cy", "width", "height")


class FootprintError(ValueError):
    """Raised when a camera ray cannot be intersected with the ground plane."""


def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Unit-quaternion (Hamilton, active, right-handed) -> 3x3 rotation matrix.

    Normalizes defensively (poses.csv values are float64 text, not guaranteed
    exactly unit-norm) and raises FootprintError on a degenerate (near-zero)
    quaternion rather than dividing by zero.
    """
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        raise FootprintError(
            f"degenerate quaternion (qx={qx}, qy={qy}, qz={qz}, qw={qw}) has near-zero norm"
        )
    s = 2.0 / n
    wx, wy, wz = s * qw * qx, s * qw * qy, s * qw * qz
    xx, xy, xz = s * qx * qx, s * qx * qy, s * qx * qz
    yy, yz, zz = s * qy * qy, s * qy * qz, s * qz * qz
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ]
    )


def _validate_intrinsics(camera_intrinsics: dict) -> Tuple[float, float, float, float, float, float]:
    missing = [k for k in REQUIRED_INTRINSICS_KEYS if k not in camera_intrinsics]
    if missing:
        raise FootprintError(f"camera_intrinsics missing keys: {missing}")
    fx = float(camera_intrinsics["fx"])
    fy = float(camera_intrinsics["fy"])
    cx = float(camera_intrinsics["cx"])
    cy = float(camera_intrinsics["cy"])
    width = float(camera_intrinsics["width"])
    height = float(camera_intrinsics["height"])
    if fx == 0.0 or fy == 0.0:
        raise FootprintError(f"camera_intrinsics fx/fy must be nonzero (fx={fx}, fy={fy})")
    if width <= 0.0 or height <= 0.0:
        raise FootprintError(f"camera_intrinsics width/height must be positive (got {width}x{height})")
    return fx, fy, cx, cy, width, height


def _corner_pixels(width: float, height: float) -> Sequence[Tuple[float, float]]:
    # Cyclic image-space order (top-left, top-right, bottom-right,
    # bottom-left) -- central projection onto a plane in front of the
    # camera preserves this cyclic order, so the projected quadrilateral is
    # a simple (non-self-intersecting) polygon.
    return ((0.0, 0.0), (width, 0.0), (width, height), (0.0, height))


def _pixel_ray_local(u: float, v: float, fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    return np.array([(u - cx) / fx, -(v - cy) / fy, -1.0])


def frame_footprint(camera_intrinsics: dict, pose: PoseRow, ground_z: float = 0.0) -> Polygon:
    """Project one frame's four image-corner rays onto the ground plane z=ground_z.

    Returns a shapely Polygon in the same local ENU (x, y) frame as
    poses.csv and the manifest survey polygon. Raises FootprintError if any
    corner ray does not point toward the ground plane (camera not oriented
    downward enough -- e.g. horizon-level or upward-pointing), or if the
    intersection would be behind the camera.
    """
    fx, fy, cx, cy, width, height = _validate_intrinsics(camera_intrinsics)
    rotation = quaternion_to_rotation_matrix(pose.qx, pose.qy, pose.qz, pose.qw)
    cam_pos = np.array([pose.x, pose.y, pose.z])

    ground_points: List[Tuple[float, float]] = []
    for u, v in _corner_pixels(width, height):
        ray_local = _pixel_ray_local(u, v, fx, fy, cx, cy)
        ray_world = rotation @ ray_local
        dz = ray_world[2]
        if dz >= -1e-9:
            raise FootprintError(
                f"frame_idx={pose.frame_idx}: camera ray does not point toward the ground "
                f"plane (ray z-component={dz:.6g}); camera is not oriented downward enough to "
                f"produce a ground footprint"
            )
        t = (ground_z - cam_pos[2]) / dz
        if t <= 0:
            raise FootprintError(
                f"frame_idx={pose.frame_idx}: computed ground intersection is behind the "
                f"camera (t={t:.6g})"
            )
        world_point = cam_pos + t * ray_world
        ground_points.append((float(world_point[0]), float(world_point[1])))

    polygon = Polygon(ground_points)
    if not polygon.is_valid or polygon.area == 0.0:
        # Defensive fallback for a degenerate/self-intersecting quadrilateral
        # (should not occur for a normal downward-facing frame, but avoids
        # ever handing an invalid geometry to shapely's union/intersection).
        polygon = MultiPoint(ground_points).convex_hull
    return polygon


def footprints_for_poses(
    camera_intrinsics: dict, poses: Iterable[PoseRow], ground_z: float = 0.0
) -> List[Polygon]:
    """Compute the per-frame ground footprint for every pose. Raises on the first bad frame."""
    return [frame_footprint(camera_intrinsics, pose, ground_z=ground_z) for pose in poses]


def union_footprints(footprints: Iterable[Polygon]):
    """Union of per-frame footprint polygons (shapely `unary_union`).

    Returns an empty Polygon if `footprints` is empty.
    """
    footprints = list(footprints)
    if not footprints:
        return Polygon()
    return unary_union(footprints)
