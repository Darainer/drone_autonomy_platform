"""Tests for photogrammetry.dataset — the DES-004 format reader.

# TS-07 (reader side) -- Verifies: MAP-3
"""
from __future__ import annotations

import csv

import pytest
import yaml

from photogrammetry import dataset as ds


def test_load_dataset_valid(valid_dataset):
    dataset = ds.load_dataset(valid_dataset)
    assert dataset.manifest["mission_id"] == "test_mission_0001"
    assert dataset.manifest["format_version"] == 1
    assert len(dataset.poses) == 5
    assert [p.frame_idx for p in dataset.poses] == [0, 1, 2, 3, 4]


def test_load_manifest_only_partial_present_is_invalid(dataset_factory):
    # TS-07(c): delete the manifest, leave only manifest.yaml.part.
    dataset_dir = dataset_factory(finalize_manifest=False)
    assert (dataset_dir / "manifest.yaml.part").is_file()
    assert not (dataset_dir / "manifest.yaml").exists()
    with pytest.raises(ds.DatasetError, match="partial manifest"):
        ds.load_manifest(dataset_dir)


def test_load_manifest_missing_entirely(tmp_path):
    dataset_dir = tmp_path / "survey_empty"
    dataset_dir.mkdir()
    with pytest.raises(ds.DatasetError):
        ds.load_manifest(dataset_dir)


def test_manifest_missing_required_key_rejected(valid_dataset):
    manifest_path = valid_dataset / "manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text())
    del manifest["altitude"]
    manifest_path.write_text(yaml.safe_dump(manifest))
    with pytest.raises(ds.DatasetError, match="altitude"):
        ds.load_manifest(valid_dataset)


def test_manifest_missing_camera_intrinsics_key_rejected(valid_dataset):
    manifest_path = valid_dataset / "manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text())
    del manifest["camera_intrinsics"]["fx"]
    manifest_path.write_text(yaml.safe_dump(manifest))
    with pytest.raises(ds.DatasetError, match="camera_intrinsics"):
        ds.load_manifest(valid_dataset)


def test_read_poses_by_name_and_contiguous(valid_dataset):
    poses = ds.read_poses(valid_dataset)
    assert [p.frame_idx for p in poses] == [0, 1, 2, 3, 4]
    ds.validate_frame_idx_contiguous(poses, start=0)
    # image_filename is derived from frame_idx/stamp_ns per DES-004 naming.
    assert poses[0].image_filename == f"000000_{poses[0].stamp_ns}.jpg"


def test_read_poses_missing_required_column_rejected(valid_dataset):
    poses_path = valid_dataset / "poses.csv"
    with open(poses_path) as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fieldnames = [c for c in reader.fieldnames if c != "sync_err_ms"]
    with open(poses_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            row.pop("sync_err_ms", None)
            writer.writerow(row)

    with pytest.raises(ds.DatasetError, match="sync_err_ms"):
        ds.read_poses(valid_dataset)


def test_validate_frame_idx_contiguous_detects_gap(valid_dataset):
    poses = ds.read_poses(valid_dataset)
    poses[-1].frame_idx += 1  # introduce a gap
    with pytest.raises(ds.DatasetError, match="contiguous"):
        ds.validate_frame_idx_contiguous(poses, start=0)


def test_validate_frame_idx_contiguous_honors_documented_start(valid_dataset):
    poses = ds.read_poses(valid_dataset)
    for p in poses:
        p.frame_idx += 10  # dataset documents a non-zero start
    # Explicit start honored, and default (first row's index) also works.
    ds.validate_frame_idx_contiguous(poses, start=10)
    ds.validate_frame_idx_contiguous(poses)


# --- FE-1 forward-compatibility -------------------------------------------------


def test_fe1_extra_manifest_key_is_ignored(dataset_factory):
    dataset_dir = dataset_factory(extra_manifest_key="rtk_fix_type")
    dataset = ds.load_dataset(dataset_dir)
    assert dataset.manifest["rtk_fix_type"] == "future_value"


def test_fe1_extra_pose_column_is_ignored(dataset_factory):
    dataset_dir = dataset_factory(extra_pose_column="rtk_covariance")
    poses = ds.read_poses(dataset_dir)
    assert len(poses) == 5
    assert poses[0].extra["rtk_covariance"] == "future_value"


def test_null_gnss_row_reads_as_none(dataset_factory):
    # DES-004 D4 / TP-002 TS-05: up to 1% of rows may have a null GNSS fix.
    # An empty lat/lon/alt_amsl/gnss_stamp_ns cell must parse to None, not raise,
    # and must not affect the other rows.
    dataset_dir = dataset_factory(num_frames=5, null_gnss_frames={2})
    poses = ds.read_poses(dataset_dir)
    assert len(poses) == 5

    null_row = poses[2]
    assert null_row.lat is None
    assert null_row.lon is None
    assert null_row.alt_amsl is None
    assert null_row.gnss_stamp_ns is None
    # Required kinematic columns on the same row are still parsed.
    assert null_row.frame_idx == 2
    assert null_row.qw == 1.0

    for other in (poses[0], poses[1], poses[3], poses[4]):
        assert other.lat is not None
        assert other.lon is not None
        assert other.gnss_stamp_ns is not None


def test_required_kinematic_column_empty_still_raises(dataset_factory):
    # An empty REQUIRED (non-GNSS) column must still hard-fail.
    dataset_dir = dataset_factory()
    poses_path = dataset_dir / "poses.csv"
    with open(poses_path) as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        rows = list(reader)
    rows[1]["x"] = ""  # blank a required column
    with open(poses_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    with pytest.raises(ds.DatasetError):
        ds.read_poses(dataset_dir)


def test_fe1_newer_format_version_still_reads(dataset_factory):
    dataset_dir = dataset_factory()
    manifest_path = dataset_dir / "manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text())
    manifest["format_version"] = 2
    manifest_path.write_text(yaml.safe_dump(manifest))

    manifest = ds.load_manifest(dataset_dir)
    assert manifest["format_version"] == 2
