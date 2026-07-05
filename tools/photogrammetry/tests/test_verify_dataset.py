"""Tests for verify_dataset.py — the standalone DES-004 checksum/manifest validator.

# TS-07 -- Verifies: MAP-3

Procedure (per TS-07):
  (a) build a valid dataset -> verify_dataset.py exits 0.
  (b) corrupt one image byte -> exit != 0, output names the corrupt file.
  (c) delete the manifest (only images/poses or a .part remain) -> reported invalid.
  Also FE-1: extra unknown manifest key / poses.csv column -> still succeeds.
"""
from __future__ import annotations

import csv
import pathlib
import subprocess
import sys

import yaml

import verify_dataset as vd
from photogrammetry.dataset import sha256_of_file

VERIFY_SCRIPT = pathlib.Path(__file__).resolve().parent.parent / "verify_dataset.py"


def _run_cli(dataset_dir) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(VERIFY_SCRIPT), str(dataset_dir)],
        capture_output=True,
        text=True,
    )


def test_ts07a_valid_dataset_exits_zero_via_cli(valid_dataset):
    result = _run_cli(valid_dataset)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "OK" in result.stdout


def test_ts07a_valid_dataset_reports_no_errors_via_api(valid_dataset):
    assert vd.verify_dataset(valid_dataset) == []


def test_ts07b_corrupted_image_byte_names_the_file(valid_dataset):
    image_path = next((valid_dataset / "images").glob("*.jpg"))
    with open(image_path, "r+b") as f:
        original = f.read(1)
        f.seek(0)
        f.write(bytes([original[0] ^ 0xFF]))

    result = _run_cli(valid_dataset)
    assert result.returncode != 0
    combined_output = result.stdout + result.stderr
    assert image_path.name in combined_output


def test_ts07c_deleted_manifest_only_part_remains_is_invalid(dataset_factory):
    dataset_dir = dataset_factory(finalize_manifest=False)
    assert (dataset_dir / "manifest.yaml.part").is_file()
    assert not (dataset_dir / "manifest.yaml").exists()

    result = _run_cli(dataset_dir)
    assert result.returncode != 0
    assert "INVALID" in result.stdout
    assert "partial manifest" in (result.stdout + result.stderr)


def test_ts07c_manifest_and_part_both_absent_is_invalid(dataset_factory):
    dataset_dir = dataset_factory(finalize_manifest=False)
    (dataset_dir / "manifest.yaml.part").unlink()

    result = _run_cli(dataset_dir)
    assert result.returncode != 0


def test_fe1_extra_manifest_key_and_pose_column_still_valid(dataset_factory):
    dataset_dir = dataset_factory(
        extra_manifest_key="rtk_fix_type", extra_pose_column="rtk_covariance"
    )
    result = _run_cli(dataset_dir)
    assert result.returncode == 0, result.stdout + result.stderr


def test_missing_referenced_image_is_reported(valid_dataset):
    image_path = next((valid_dataset / "images").glob("*.jpg"))
    image_path.unlink()

    errors = vd.verify_dataset(valid_dataset)
    assert errors  # not valid
    assert any(image_path.name in e for e in errors)


def test_collect_warnings_flags_image_dims_manifest_mismatch(dataset_factory):
    # Default dataset_factory datasets use 8x8 placeholder JPEGs (conftest
    # `_make_tiny_jpeg`) against the default 640x480 camera_intrinsics --
    # they mismatch naturally, no special construction needed (F-6(b)).
    dataset_dir = dataset_factory(mission_id="dims_mismatch")

    warnings = vd.collect_warnings(dataset_dir)

    assert len(warnings) == 1
    assert "8x8" in warnings[0]
    assert "640x480" in warnings[0]

    # The mismatch is a WARNING, not an error -- the dataset is still valid
    # and the CLI still exits 0.
    assert vd.verify_dataset(dataset_dir) == []
    result = _run_cli(dataset_dir)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "WARNING" in (result.stdout + result.stderr)
    assert "8x8" in (result.stdout + result.stderr)


def test_collect_warnings_empty_when_dims_match(dataset_factory):
    # A camera_intrinsics block that matches the actual 8x8 placeholder
    # JPEGs -> no mismatch -> no warnings.
    dataset_dir = dataset_factory(
        mission_id="dims_consistent",
        camera_intrinsics={"fx": 6.0, "fy": 6.0, "cx": 4.0, "cy": 4.0, "width": 8, "height": 8},
    )

    assert vd.collect_warnings(dataset_dir) == []

    result = _run_cli(dataset_dir)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "WARNING" not in (result.stdout + result.stderr)


def test_collect_warnings_empty_when_manifest_unloadable(dataset_factory):
    dataset_dir = dataset_factory(mission_id="no_manifest", finalize_manifest=False)
    assert vd.collect_warnings(dataset_dir) == []


def test_collect_warnings_empty_when_first_image_missing(dataset_factory):
    dataset_dir = dataset_factory(mission_id="missing_first_image")
    poses = list(csv.DictReader(open(dataset_dir / "poses.csv")))
    first_image = dataset_dir / "images" / f"{int(poses[0]['frame_idx']):06d}_{poses[0]['stamp_ns']}.jpg"
    first_image.unlink()

    assert vd.collect_warnings(dataset_dir) == []


def test_frame_idx_gap_is_reported(dataset_factory):
    dataset_dir = dataset_factory(mission_id="gap_mission")

    poses_path = dataset_dir / "poses.csv"
    with open(poses_path) as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        rows = list(reader)
    rows[-1]["frame_idx"] = str(int(rows[-1]["frame_idx"]) + 1)
    with open(poses_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    # Keep the checksum check isolated from the contiguity check by updating
    # the manifest's recorded poses.csv hash to match the edited file.
    manifest_path = dataset_dir / "manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text())
    manifest["sha256"]["poses.csv"] = sha256_of_file(poses_path)
    manifest_path.write_text(yaml.safe_dump(manifest))

    errors = vd.verify_dataset(dataset_dir)
    assert any("contiguous" in e for e in errors)
