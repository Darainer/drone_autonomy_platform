"""Shared pytest fixtures for the photogrammetry tests.

Builds a small, synthetic-but-schema-valid DES-004 dataset in a tmp dir for
each test that needs one -- real (tiny) JPEGs via Pillow, a poses.csv with
the full required column set, and a manifest.yaml with correct sha256
hashes. Unit tests must be fully self-contained; none of this depends on any
checked-in fixture dataset (the checked-in reference sample dataset is a
later task, DES-005 T3.4).
"""
from __future__ import annotations

import csv
import hashlib
import os
import pathlib
import sys
from typing import Optional

# Make the top-level `photogrammetry` package and `verify_dataset.py` module
# importable regardless of how pytest was invoked (robust even if not run as
# `python -m pytest` from this directory).
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import pytest
import yaml
from PIL import Image

from photogrammetry.dataset import REQUIRED_POSE_COLUMNS

DEFAULT_MISSION_ID = "test_mission_0001"
_COLORS = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255)]


def _sha256_of_file(path: pathlib.Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _make_tiny_jpeg(path: pathlib.Path, color) -> None:
    Image.new("RGB", (8, 8), color=color).save(path, format="JPEG", quality=95)


def _pose_row(
    frame_idx: int,
    stamp_ns: int,
    extra_column: Optional[str] = None,
    null_gnss: bool = False,
) -> dict:
    row = {
        "frame_idx": frame_idx,
        "stamp_ns": stamp_ns,
        "x": 1.0 + frame_idx,
        "y": 2.0 + frame_idx,
        "z": 50.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0,
        "qw": 1.0,
        "pose_stamp_ns": stamp_ns + 1_000_000,
        "lat": 37.7 + frame_idx * 1e-5,
        "lon": -122.4 + frame_idx * 1e-5,
        "alt_amsl": 100.0,
        "gnss_stamp_ns": stamp_ns + 2_000_000,
        "sync_err_ms": 5.0,
    }
    if null_gnss:
        # DES-004 D4 / TS-05: a frame with no recent-enough GNSS fix leaves
        # the GNSS columns empty.
        row["lat"] = ""
        row["lon"] = ""
        row["alt_amsl"] = ""
        row["gnss_stamp_ns"] = ""
    if extra_column:
        row[extra_column] = "future_value"
    return row


def build_dataset(
    root: pathlib.Path,
    mission_id: str = DEFAULT_MISSION_ID,
    num_frames: int = 5,
    extra_manifest_key: Optional[str] = None,
    extra_pose_column: Optional[str] = None,
    null_gnss_frames: Optional[set] = None,
    finalize_manifest: bool = True,
    polygon: Optional[list] = None,
    camera_intrinsics: Optional[dict] = None,
    altitude: Optional[float] = None,
    pose_row_fn=None,
) -> pathlib.Path:
    """Build a small but schema-valid DES-004 dataset under `root`.

    `polygon`, `camera_intrinsics`, and `altitude` override the
    corresponding manifest fields (defaults unchanged from T3.1: a 10x10
    square, a fixed intrinsics dict, and altitude 50.0) -- used by T3.2
    tests that need a specific survey polygon / camera model to build a
    dataset whose predicted footprint coverage is controlled (e.g. a
    REF-RECT-covering or deliberately under-covering dataset).

    `pose_row_fn`, if given, is called as `pose_row_fn(frame_idx, stamp_ns)`
    and must return a complete poses.csv row dict (all REQUIRED_POSE_COLUMNS
    keys); it replaces the default synthetic pose generator entirely for
    this dataset (so `extra_pose_column`/`null_gnss_frames` are ignored when
    `pose_row_fn` is provided -- the caller's row already owns all columns).

    Returns the dataset directory (`root/survey_<mission_id>`).
    """
    dataset_dir = root / f"survey_{mission_id}"
    images_dir = dataset_dir / "images"
    images_dir.mkdir(parents=True)

    stamps = [1_700_000_000_000_000_000 + i * 500_000_000 for i in range(num_frames)]

    image_relpaths = []
    for frame_idx, stamp_ns in enumerate(stamps):
        filename = f"{frame_idx:06d}_{stamp_ns}.jpg"
        _make_tiny_jpeg(images_dir / filename, _COLORS[frame_idx % len(_COLORS)])
        image_relpaths.append(f"images/{filename}")

    fieldnames = list(REQUIRED_POSE_COLUMNS)
    if extra_pose_column:
        fieldnames = fieldnames + [extra_pose_column]

    poses_path = dataset_dir / "poses.csv"
    with open(poses_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        null_gnss_frames = null_gnss_frames or set()
        for frame_idx, stamp_ns in enumerate(stamps):
            if pose_row_fn is not None:
                row = pose_row_fn(frame_idx, stamp_ns)
            else:
                row = _pose_row(
                    frame_idx,
                    stamp_ns,
                    extra_pose_column,
                    null_gnss=frame_idx in null_gnss_frames,
                )
            writer.writerow(row)

    sha256_map = {relpath: _sha256_of_file(dataset_dir / relpath) for relpath in image_relpaths}
    sha256_map["poses.csv"] = _sha256_of_file(poses_path)

    manifest = {
        "format_version": 1,
        "mission_id": mission_id,
        "polygon": polygon if polygon is not None else [[0.0, 0.0], [0.0, 10.0], [10.0, 10.0], [10.0, 0.0]],
        "altitude": altitude if altitude is not None else 50.0,
        "overlaps": {"forward": 80, "side": 70},
        "start_utc": "2026-07-05T12:00:00Z",
        "end_utc": "2026-07-05T12:05:00Z",
        "frame_count": num_frames,
        "camera_intrinsics": camera_intrinsics
        if camera_intrinsics is not None
        else {
            "fx": 600.0,
            "fy": 600.0,
            "cx": 320.0,
            "cy": 240.0,
            "width": 640,
            "height": 480,
        },
        "sha256": sha256_map,
    }
    if extra_manifest_key:
        manifest[extra_manifest_key] = "future_value"

    # Mimic the real recorder: write to manifest.yaml.part, then atomically
    # rename to manifest.yaml on "disarm" (DES-004).
    manifest_part_path = dataset_dir / "manifest.yaml.part"
    with open(manifest_part_path, "w") as f:
        yaml.safe_dump(manifest, f)

    if finalize_manifest:
        os.rename(manifest_part_path, dataset_dir / "manifest.yaml")

    return dataset_dir


@pytest.fixture
def dataset_factory(tmp_path):
    """Factory fixture: call with kwargs (see build_dataset) to build a synthetic dataset."""

    def _factory(**kwargs):
        return build_dataset(tmp_path, **kwargs)

    return _factory


@pytest.fixture
def valid_dataset(dataset_factory) -> pathlib.Path:
    """A default valid, finalized, schema-complete synthetic dataset (5 frames)."""
    return dataset_factory()
