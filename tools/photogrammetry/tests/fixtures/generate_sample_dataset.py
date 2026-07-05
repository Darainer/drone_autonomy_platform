#!/usr/bin/env python3
"""generate_sample_dataset.py — F2 reference sample dataset generator (TP-002 F2).

Generates a synthetic-but-schema-valid DES-004 dataset flown as a
**lawnmower** (boustrophedon) survey pattern over a rectangular polygon,
with a nadir (straight-down, identity-quaternion) camera model chosen to
match the TP-002 "common parameters":

    altitude 40 m, forward_overlap 0.75, side_overlap 0.60,
    capture_rate 2 Hz, OAK-D FOV ~69 deg / 55 deg
    (nadir ground footprint ~= 55 m x 42 m at 40 m AGL)

The default call (no arguments) reproduces the checked-in F2 fixture: a
~20-frame dataset over `REF-RECT` (TP-002 F1: 100 m x 80 m rectangle),
written to `tests/fixtures/survey_ref_rect_sample/`.

# Coverage guarantee
The camera is nadir (identity quaternion: qx=qy=qz=0, qw=1), so per
`photogrammetry/footprint.py`'s documented frame convention, each frame's
ground footprint is the axis-aligned box
    [x - footprint_width/2, x + footprint_width/2]
    x [y - footprint_height/2, y + footprint_height/2]
centered on the waypoint (x, y). Line spacing (Y) and frame spacing (X) are
each derived from the requested overlap fractions
(`spacing = footprint_dim * (1 - overlap)`) and rounded DOWN to the nearest
count that fits evenly across the rectangle (never up), so the achieved
spacing is always <= the nominal max spacing -- i.e. overlap only ever
Increases versus the nominal request. With margins of footprint_dim/2 at
each edge, the lines/frames union provably tiles the full rectangle
(verified empirically by `tests/test_reference_dataset.py`'s
`run_pipeline.py --mode check` coverage_pct >= 95% assertion -- TS-08/TS-10).

# Scaling to the TS-10 1000-frame Jetson HIL dataset
`num_lines` and `frames_per_line` are auto-derived from
(`rect_width`, `rect_height`, `altitude`, `camera_intrinsics`, overlaps) --
they are NOT independent knobs you need to hand-tune. To grow the ~20-frame
F2 sample into the "1000-frame synthetic dataset (F2 generator scaled up)"
TS-10 asks for, keep `altitude`/overlaps/camera model fixed (so the
per-frame footprint stays the TP-002 common-parameters footprint) and scale
up the rectangle instead, e.g.:

    generate_sample_dataset(
        output_dir=pathlib.Path("/tmp/ts10_dataset"),
        mission_id="ts10_jetson_hil_1000f",
        rect_width=610.0, rect_height=420.0,
    )

which yields ~1000 frames (42 frames/line x 24 lines) at the same 40 m
altitude / 0.75 / 0.60 overlaps / footprint model as F2 -- i.e. the same
generator, only the survey area changed. This larger dataset is NOT checked
in (only the ~20-frame F2 instance is, per TP-002); TS-10 generates it
on-demand on the Jetson HIL bench.

Usage as a script (regenerates the checked-in F2 fixture in place):

    python tests/fixtures/generate_sample_dataset.py

Usage as a library (e.g. from a test or another generator call):

    from generate_sample_dataset import generate_sample_dataset
    generate_sample_dataset(output_dir=tmp_path, mission_id="my_ds")
"""
from __future__ import annotations

import argparse
import csv
import datetime
import hashlib
import math
import pathlib
import sys
from typing import Any, Dict, List, Optional, Tuple

import yaml
from PIL import Image

# Make the `photogrammetry` package importable regardless of invocation cwd
# (this file lives at tools/photogrammetry/tests/fixtures/; the package root
# is two levels up). Reuse dataset.py's column list + hashing rather than
# duplicating them (T3.4 scope: generator only, no changes to dataset.py).
_PACKAGE_ROOT = pathlib.Path(__file__).resolve().parent.parent.parent
if str(_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_ROOT))

from photogrammetry.dataset import REQUIRED_POSE_COLUMNS, sha256_of_file  # noqa: E402

# TP-002 F1: REF-RECT default extent (100 m x 80 m rectangle), local ENU
# meters, matching the `REF_RECT` constant used by
# tests/test_run_pipeline_check.py.
DEFAULT_RECT_WIDTH = 100.0
DEFAULT_RECT_HEIGHT = 80.0

# TP-002 "common parameters".
DEFAULT_ALTITUDE = 40.0
DEFAULT_FORWARD_OVERLAP = 0.75
DEFAULT_SIDE_OVERLAP = 0.60
DEFAULT_CAPTURE_RATE_HZ = 2.0

# OAK-D-like sensor: image size chosen together with fx/fy (below) to land
# on the TP-002 "~55 m x 42 m at 40 m AGL" nadir footprint.
DEFAULT_IMAGE_WIDTH = 640
DEFAULT_IMAGE_HEIGHT = 480
DEFAULT_FOOTPRINT_WIDTH_M = 55.0  # at DEFAULT_ALTITUDE
DEFAULT_FOOTPRINT_HEIGHT_M = 42.0  # at DEFAULT_ALTITUDE

DEFAULT_MISSION_ID = "ref_rect_sample"

# Base lat/lon/date used only to populate the (non-gated) GNSS columns with
# plausible, non-null values -- coverage/check-mode math (footprint.py,
# coverage.py) uses only the local x/y/z + camera intrinsics, never lat/lon.
_BASE_LAT_DEG = 37.700000
_BASE_LON_DEG = -122.400000
_METERS_PER_DEG_LAT = 111_320.0
_BASE_START_UTC = datetime.datetime(2026, 1, 15, 9, 0, 0, tzinfo=datetime.timezone.utc)

_TINY_JPEG_PX = 8  # keep checked-in images tiny; must still be valid JPEGs.
_COLORS = [(200, 30, 30), (30, 200, 30), (30, 30, 200), (200, 200, 30), (30, 200, 200)]


def default_camera_intrinsics(
    altitude: float = DEFAULT_ALTITUDE,
    footprint_width_m: float = DEFAULT_FOOTPRINT_WIDTH_M,
    footprint_height_m: float = DEFAULT_FOOTPRINT_HEIGHT_M,
    width: int = DEFAULT_IMAGE_WIDTH,
    height: int = DEFAULT_IMAGE_HEIGHT,
) -> Dict[str, Any]:
    """Derive DES-004 `camera_intrinsics` giving the requested nadir ground footprint.

    Per `footprint.py`'s nadir camera model, footprint_width = altitude *
    width / fx (and symmetrically for height/fy), so fx/fy are solved for
    directly from the desired footprint size at `altitude`. cx/cy are the
    image center (no distortion modeled, matching footprint.py's pinhole
    assumption).
    """
    fx = altitude * width / footprint_width_m
    fy = altitude * height / footprint_height_m
    return {
        "fx": fx,
        "fy": fy,
        "cx": width / 2.0,
        "cy": height / 2.0,
        "width": width,
        "height": height,
    }


def _lawnmower_axis_counts(
    extent: float, footprint_dim: float, overlap: float
) -> Tuple[int, float]:
    """Derive (count, spacing) of waypoints along one axis of the rectangle.

    Waypoints are placed with a footprint_dim/2 margin from each edge so the
    footprint never has to extend past the rectangle to reach it, and
    spacing is `extent_between_first_and_last / (count - 1)`, which is
    always <= `footprint_dim * (1 - overlap)` (the nominal max spacing for
    the requested overlap) because `count` is rounded UP from that nominal
    spacing. A single-waypoint axis (footprint already >= extent) returns
    (1, 0.0), placing that waypoint at the rectangle's center on that axis.
    """
    if footprint_dim >= extent:
        return 1, 0.0
    nominal_spacing = footprint_dim * (1.0 - overlap)
    span = extent - footprint_dim  # distance between the first and last waypoint
    gaps = max(1, math.ceil(span / nominal_spacing))
    count = gaps + 1
    spacing = span / gaps
    return count, spacing


def _lawnmower_waypoints(
    rect_width: float,
    rect_height: float,
    footprint_width_m: float,
    footprint_height_m: float,
    forward_overlap: float,
    side_overlap: float,
    num_lines: Optional[int] = None,
    frames_per_line: Optional[int] = None,
) -> List[Tuple[float, float]]:
    """Build a boustrophedon (serpentine) lawnmower waypoint list over the rectangle.

    Lines run along X (the "forward"/along-track direction); line spacing
    along Y uses `side_overlap`. Frame spacing along each line (X) uses
    `forward_overlap`. `num_lines`/`frames_per_line` override the
    auto-derived counts if given (still evenly spaced across the rectangle
    with the same edge margins) -- see module docstring for scaling this to
    the TS-10 1000-frame dataset by growing the rectangle instead.
    """
    if frames_per_line is None or frames_per_line < 1:
        frames_per_line, x_spacing = _lawnmower_axis_counts(
            rect_width, footprint_width_m, forward_overlap
        )
    else:
        x_span = max(rect_width - footprint_width_m, 0.0)
        x_spacing = x_span / (frames_per_line - 1) if frames_per_line > 1 else 0.0

    if num_lines is None or num_lines < 1:
        num_lines, y_spacing = _lawnmower_axis_counts(rect_height, footprint_height_m, side_overlap)
    else:
        y_span = max(rect_height - footprint_height_m, 0.0)
        y_spacing = y_span / (num_lines - 1) if num_lines > 1 else 0.0

    x_margin = min(footprint_width_m / 2.0, rect_width / 2.0)
    y_margin = min(footprint_height_m / 2.0, rect_height / 2.0)

    waypoints: List[Tuple[float, float]] = []
    for line_idx in range(num_lines):
        y = y_margin + line_idx * y_spacing
        xs = [x_margin + i * x_spacing for i in range(frames_per_line)]
        if line_idx % 2 == 1:
            xs = list(reversed(xs))  # boustrophedon: alternate direction each line
        for x in xs:
            waypoints.append((x, y))
    return waypoints


def _make_tiny_jpeg(path: pathlib.Path, color) -> None:
    Image.new("RGB", (_TINY_JPEG_PX, _TINY_JPEG_PX), color=color).save(path, format="JPEG", quality=95)


def _latlon_for_xy(x: float, y: float) -> Tuple[float, float]:
    """Flat-earth local-ENU -> lat/lon approximation (fine for a synthetic fixture)."""
    lat = _BASE_LAT_DEG + y / _METERS_PER_DEG_LAT
    meters_per_deg_lon = _METERS_PER_DEG_LAT * math.cos(math.radians(_BASE_LAT_DEG))
    lon = _BASE_LON_DEG + x / meters_per_deg_lon
    return lat, lon


def generate_sample_dataset(
    output_dir: pathlib.Path,
    mission_id: str = DEFAULT_MISSION_ID,
    rect_width: float = DEFAULT_RECT_WIDTH,
    rect_height: float = DEFAULT_RECT_HEIGHT,
    altitude: float = DEFAULT_ALTITUDE,
    forward_overlap: float = DEFAULT_FORWARD_OVERLAP,
    side_overlap: float = DEFAULT_SIDE_OVERLAP,
    capture_rate_hz: float = DEFAULT_CAPTURE_RATE_HZ,
    camera_intrinsics: Optional[Dict[str, Any]] = None,
    num_lines: Optional[int] = None,
    frames_per_line: Optional[int] = None,
    sync_err_ms: float = 5.0,
) -> pathlib.Path:
    """Generate a valid DES-004 dataset (F2-style) under `output_dir/survey_<mission_id>/`.

    Returns the dataset directory. See module docstring for the coverage
    guarantee and for scaling this up to the TS-10 1000-frame dataset via
    `rect_width`/`rect_height` (frame count is derived, not a free knob, by
    design -- so growing the survey area is what grows the dataset).
    """
    if camera_intrinsics is None:
        camera_intrinsics = default_camera_intrinsics(altitude=altitude)

    footprint_width_m = altitude * camera_intrinsics["width"] / camera_intrinsics["fx"]
    footprint_height_m = altitude * camera_intrinsics["height"] / camera_intrinsics["fy"]

    waypoints = _lawnmower_waypoints(
        rect_width,
        rect_height,
        footprint_width_m,
        footprint_height_m,
        forward_overlap,
        side_overlap,
        num_lines=num_lines,
        frames_per_line=frames_per_line,
    )
    num_frames = len(waypoints)

    dataset_dir = pathlib.Path(output_dir) / f"survey_{mission_id}"
    images_dir = dataset_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)

    dt_ns = int(round(1.0e9 / capture_rate_hz))
    base_stamp_ns = 1_700_000_000_000_000_000
    stamps = [base_stamp_ns + i * dt_ns for i in range(num_frames)]

    image_relpaths: List[str] = []
    for frame_idx, stamp_ns in enumerate(stamps):
        filename = f"{frame_idx:06d}_{stamp_ns}.jpg"
        _make_tiny_jpeg(images_dir / filename, _COLORS[frame_idx % len(_COLORS)])
        image_relpaths.append(f"images/{filename}")

    poses_path = dataset_dir / "poses.csv"
    with open(poses_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(REQUIRED_POSE_COLUMNS))
        writer.writeheader()
        for frame_idx, (stamp_ns, (x, y)) in enumerate(zip(stamps, waypoints)):
            lat, lon = _latlon_for_xy(x, y)
            writer.writerow(
                {
                    "frame_idx": frame_idx,
                    "stamp_ns": stamp_ns,
                    "x": x,
                    "y": y,
                    "z": altitude,
                    "qx": 0.0,
                    "qy": 0.0,
                    "qz": 0.0,
                    "qw": 1.0,  # identity quaternion: nadir, no gimbal/yaw (footprint.py convention)
                    "pose_stamp_ns": stamp_ns + 1_000_000,
                    "lat": lat,
                    "lon": lon,
                    "alt_amsl": altitude + 100.0,
                    "gnss_stamp_ns": stamp_ns + 2_000_000,
                    "sync_err_ms": sync_err_ms,
                }
            )

    sha256_map = {relpath: sha256_of_file(dataset_dir / relpath) for relpath in image_relpaths}
    sha256_map["poses.csv"] = sha256_of_file(poses_path)

    duration_s = (num_frames - 1) / capture_rate_hz if num_frames > 1 else 0.0
    start_utc = _BASE_START_UTC
    end_utc = start_utc + datetime.timedelta(seconds=duration_s)

    polygon = [[0.0, 0.0], [rect_width, 0.0], [rect_width, rect_height], [0.0, rect_height]]

    manifest: Dict[str, Any] = {
        "format_version": 1,
        "mission_id": mission_id,
        "polygon": polygon,
        "altitude": altitude,
        "overlaps": {"forward": forward_overlap, "side": side_overlap},
        "start_utc": start_utc.strftime("%Y-%m-%dT%H:%M:%SZ"),
        "end_utc": end_utc.strftime("%Y-%m-%dT%H:%M:%SZ"),
        "frame_count": num_frames,
        "camera_intrinsics": camera_intrinsics,
        "sha256": sha256_map,
    }

    # Mimic the real recorder (DES-004): write manifest.yaml.part, then
    # atomically rename to manifest.yaml on "disarm".
    manifest_part_path = dataset_dir / "manifest.yaml.part"
    with open(manifest_part_path, "w") as f:
        yaml.safe_dump(manifest, f, sort_keys=False)
    manifest_part_path.rename(dataset_dir / "manifest.yaml")

    return dataset_dir


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Generate a synthetic DES-004 lawnmower-survey dataset (TP-002 F2)."
    )
    default_output_dir = pathlib.Path(__file__).resolve().parent
    parser.add_argument(
        "--output-dir",
        default=str(default_output_dir),
        help=f"Parent directory to write survey_<mission-id>/ under (default: {default_output_dir}).",
    )
    parser.add_argument("--mission-id", default=DEFAULT_MISSION_ID)
    parser.add_argument("--rect-width", type=float, default=DEFAULT_RECT_WIDTH)
    parser.add_argument("--rect-height", type=float, default=DEFAULT_RECT_HEIGHT)
    parser.add_argument("--altitude", type=float, default=DEFAULT_ALTITUDE)
    parser.add_argument("--forward-overlap", type=float, default=DEFAULT_FORWARD_OVERLAP)
    parser.add_argument("--side-overlap", type=float, default=DEFAULT_SIDE_OVERLAP)
    parser.add_argument("--capture-rate-hz", type=float, default=DEFAULT_CAPTURE_RATE_HZ)
    args = parser.parse_args(argv)

    dataset_dir = generate_sample_dataset(
        output_dir=pathlib.Path(args.output_dir),
        mission_id=args.mission_id,
        rect_width=args.rect_width,
        rect_height=args.rect_height,
        altitude=args.altitude,
        forward_overlap=args.forward_overlap,
        side_overlap=args.side_overlap,
        capture_rate_hz=args.capture_rate_hz,
    )
    print(f"generated {dataset_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
