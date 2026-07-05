"""Tests for photogrammetry.footprint — camera model + ground footprint + union.

Design note (see photogrammetry/footprint.py module docstring): an identity
pose quaternion (qx=qy=qz=0, qw=1) is defined as a nadir-pointing camera --
the fixed downward-boresight convention used since DES-004 records no
separate camera extrinsic.
"""
from __future__ import annotations

import math

import pytest
from shapely.geometry import Polygon

from photogrammetry import footprint as fp
from photogrammetry.dataset import PoseRow

NADIR_INTRINSICS = {"fx": 100.0, "fy": 100.0, "cx": 100.0, "cy": 100.0, "width": 200, "height": 200}


def _pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0, frame_idx=0) -> PoseRow:
    return PoseRow(
        frame_idx=frame_idx,
        stamp_ns=0,
        x=x,
        y=y,
        z=z,
        qx=qx,
        qy=qy,
        qz=qz,
        qw=qw,
        pose_stamp_ns=0,
        lat=None,
        lon=None,
        alt_amsl=None,
        gnss_stamp_ns=None,
        sync_err_ms=0.0,
    )


def test_identity_quaternion_is_nadir_and_centered():
    altitude = 50.0
    pose = _pose(x=10.0, y=20.0, z=altitude)
    poly = fp.frame_footprint(NADIR_INTRINSICS, pose)

    assert isinstance(poly, Polygon)
    assert poly.is_valid
    centroid = poly.centroid
    assert centroid.x == pytest.approx(10.0, abs=1e-6)
    assert centroid.y == pytest.approx(20.0, abs=1e-6)


def test_nadir_footprint_size_matches_ground_sample_scaling():
    # Ground footprint side (m) = altitude * width_px / fx (standard nadir
    # pinhole-on-a-plane scaling); with fx=fy=100, width=height=200 the
    # footprint is a square of side 2*altitude.
    altitude = 30.0
    pose = _pose(x=0.0, y=0.0, z=altitude)
    poly = fp.frame_footprint(NADIR_INTRINSICS, pose)

    minx, miny, maxx, maxy = poly.bounds
    expected_side = altitude * NADIR_INTRINSICS["width"] / NADIR_INTRINSICS["fx"]
    assert (maxx - minx) == pytest.approx(expected_side, rel=1e-6)
    assert (maxy - miny) == pytest.approx(expected_side, rel=1e-6)
    assert poly.area == pytest.approx(expected_side * expected_side, rel=1e-6)


def test_camera_not_pointing_downward_raises():
    # A 90-degree rotation about the X axis turns the fixed downward
    # boresight into a horizontal ray -- it never reaches the ground plane.
    half = math.sqrt(0.5)
    pose = _pose(x=0.0, y=0.0, z=50.0, qx=half, qy=0.0, qz=0.0, qw=half)
    with pytest.raises(fp.FootprintError):
        fp.frame_footprint(NADIR_INTRINSICS, pose)


def test_union_of_overlapping_footprints_is_less_than_sum():
    altitude = 30.0
    # Footprint side = 60m; offset the second pose by 20m so the two
    # footprints overlap substantially but are not identical.
    pose_a = _pose(x=0.0, y=0.0, z=altitude, frame_idx=0)
    pose_b = _pose(x=20.0, y=0.0, z=altitude, frame_idx=1)

    poly_a = fp.frame_footprint(NADIR_INTRINSICS, pose_a)
    poly_b = fp.frame_footprint(NADIR_INTRINSICS, pose_b)
    union = fp.union_footprints([poly_a, poly_b])

    assert union.area < poly_a.area + poly_b.area
    assert union.area > max(poly_a.area, poly_b.area)


def test_union_of_empty_list_is_empty_geometry():
    union = fp.union_footprints([])
    assert union.is_empty


def test_footprints_for_poses_matches_individual_calls():
    altitude = 25.0
    poses = [_pose(x=float(i) * 5.0, y=0.0, z=altitude, frame_idx=i) for i in range(3)]
    footprints = fp.footprints_for_poses(NADIR_INTRINSICS, poses)
    assert len(footprints) == 3
    for i, poly in enumerate(footprints):
        expected = fp.frame_footprint(NADIR_INTRINSICS, poses[i])
        assert poly.equals(expected)


def test_missing_intrinsics_key_raises():
    bad_intrinsics = dict(NADIR_INTRINSICS)
    del bad_intrinsics["fx"]
    pose = _pose(x=0.0, y=0.0, z=10.0)
    with pytest.raises(fp.FootprintError):
        fp.frame_footprint(bad_intrinsics, pose)
