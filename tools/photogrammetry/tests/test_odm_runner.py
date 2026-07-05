"""Unit tests for photogrammetry/odm_runner.py: geo.txt generation + argv shape.

# Verifies: MAP-4
"""
from __future__ import annotations

import subprocess

import numpy as np
import pytest
from PIL import Image
from PIL.TiffImagePlugin import ImageFileDirectory_v2

from photogrammetry import dataset as ds, odm_runner


def _write_geotiff_orthophoto(
    path,
    width,
    height,
    scale_x,
    scale_y,
    world_x0,
    world_y0,
    valid_box,
):
    """Write a synthetic RGBA GeoTIFF with ModelPixelScale/ModelTiepoint tags.

    `valid_box` = (min_col, min_row, max_col, max_row) inclusive pixel range
    that is marked opaque (alpha=255); everything else is transparent
    (alpha=0), mimicking an ODM nodata-transparent orthophoto. The tiepoint
    maps raster (0, 0) -> world (world_x0, world_y0) (top-left).
    """
    rgba = np.zeros((height, width, 4), dtype=np.uint8)
    rgba[:, :, :3] = 120  # arbitrary opaque-ish color for valid pixels
    min_col, min_row, max_col, max_row = valid_box
    rgba[min_row : max_row + 1, min_col : max_col + 1, 3] = 255

    img = Image.fromarray(rgba, mode="RGBA")

    info = ImageFileDirectory_v2()
    info.tagtype[odm_runner.GEOTIFF_MODEL_PIXEL_SCALE_TAG] = 12  # DOUBLE
    info.tagtype[odm_runner.GEOTIFF_MODEL_TIEPOINT_TAG] = 12
    info[odm_runner.GEOTIFF_MODEL_PIXEL_SCALE_TAG] = (scale_x, scale_y, 0.0)
    info[odm_runner.GEOTIFF_MODEL_TIEPOINT_TAG] = (0.0, 0.0, 0.0, world_x0, world_y0, 0.0)

    img.save(path, format="TIFF", tiffinfo=info)


def test_build_odm_command_is_non_interactive(tmp_path):
    """STK-1(d): the docker run argv must never request a tty/stdin attach.

    No `-i`, `-t`, or combined `-it`/`-ti` flag anywhere in the argv --
    the container must run fully unattended.
    """
    dataset_dir = tmp_path / "dataset"
    dataset_dir.mkdir()
    geo_txt_path = dataset_dir / "geo.txt"
    geo_txt_path.write_text("WGS84\n000000_1.jpg -122.4 37.7 100.0\n")

    argv = odm_runner.build_odm_command(dataset_dir, geo_txt_path)

    disallowed = {"-i", "-t", "-it", "-ti", "--interactive", "--tty"}
    assert not (disallowed & set(argv)), f"interactive flag found in argv: {argv}"
    assert "docker" in argv
    assert "run" in argv
    # The geo.txt must actually be referenced (georeferencing wired in).
    assert any("geo.txt" in arg for arg in argv)
    assert odm_runner.ODM_IMAGE in argv


def test_build_odm_command_pins_a_real_looking_odm_tag():
    # A pinned tag, not `latest` -- DES-005 D1 requires a specific version.
    assert odm_runner.ODM_IMAGE.startswith("opendronemap/odm:")
    assert odm_runner.ODM_IMAGE != "opendronemap/odm:latest"


def test_write_geo_txt_skips_null_gnss_rows(dataset_factory):
    dataset_dir = dataset_factory(
        mission_id="geo_txt_test",
        num_frames=4,
        null_gnss_frames={1},
    )
    poses = ds.read_poses(dataset_dir)

    geo_txt_path = odm_runner.write_geo_txt(dataset_dir, poses)

    assert geo_txt_path.is_file()
    lines = geo_txt_path.read_text().strip().splitlines()
    assert lines[0] == "WGS84"
    # 4 frames, 1 with null lat/lon -> 3 georeferenced image lines.
    assert len(lines) == 1 + 3
    frame_1_image = poses[1].image_filename
    assert not any(line.startswith(frame_1_image) for line in lines[1:])


def test_write_geo_txt_raises_when_no_pose_has_gnss(dataset_factory):
    dataset_dir = dataset_factory(
        mission_id="geo_txt_no_gnss",
        num_frames=2,
        null_gnss_frames={0, 1},
    )
    poses = ds.read_poses(dataset_dir)

    with pytest.raises(odm_runner.OdmRunnerError):
        odm_runner.write_geo_txt(dataset_dir, poses)


def test_products_complete_false_when_missing(tmp_path):
    dataset_dir = tmp_path / "dataset"
    dataset_dir.mkdir()
    assert odm_runner.products_complete(dataset_dir) is False


def test_products_complete_true_when_all_nonempty(tmp_path):
    dataset_dir = tmp_path / "dataset"
    products_dir = dataset_dir / "products"
    products_dir.mkdir(parents=True)
    for name in (
        odm_runner.LAZ_FILENAME,
        odm_runner.OBJ_FILENAME,
        odm_runner.MTL_FILENAME,
        odm_runner.ORTHOPHOTO_FILENAME,
    ):
        (products_dir / name).write_bytes(b"stub-bytes")

    assert odm_runner.products_complete(dataset_dir) is True


def test_products_complete_false_when_empty_file(tmp_path):
    dataset_dir = tmp_path / "dataset"
    products_dir = dataset_dir / "products"
    products_dir.mkdir(parents=True)
    for name in (
        odm_runner.LAZ_FILENAME,
        odm_runner.OBJ_FILENAME,
        odm_runner.MTL_FILENAME,
        odm_runner.ORTHOPHOTO_FILENAME,
    ):
        (products_dir / name).write_bytes(b"" if name == odm_runner.ORTHOPHOTO_FILENAME else b"x")

    assert odm_runner.products_complete(dataset_dir) is False


def test_reconstruction_footprint_reads_sidecar(tmp_path):
    dataset_dir = tmp_path / "dataset"
    vertices = [(0.0, 0.0), (100.0, 0.0), (100.0, 80.0), (0.0, 80.0)]
    odm_runner.write_footprint_sidecar(dataset_dir, vertices)

    geom = odm_runner.reconstruction_footprint(dataset_dir)
    assert geom.area == pytest.approx(100.0 * 80.0)


def test_reconstruction_footprint_raises_when_sidecar_missing(tmp_path):
    dataset_dir = tmp_path / "dataset"
    dataset_dir.mkdir()
    with pytest.raises(odm_runner.OdmRunnerError):
        odm_runner.reconstruction_footprint(dataset_dir)


def test_default_runner_closes_stdin_and_avoids_tty(monkeypatch):
    """STK-1(d): the real subprocess runner must never attach a tty/stdin.

    Patches subprocess.run to capture kwargs rather than actually invoking
    docker (unavailable in this environment).
    """
    captured = {}

    def fake_subprocess_run(argv, **kwargs):
        captured["argv"] = argv
        captured["kwargs"] = kwargs
        return subprocess.CompletedProcess(argv, returncode=0, stdout="", stderr="")

    monkeypatch.setattr(subprocess, "run", fake_subprocess_run)

    argv = ["docker", "run", "--rm", odm_runner.ODM_IMAGE]
    result = odm_runner._default_runner(argv)

    assert result.returncode == 0
    assert captured["kwargs"]["stdin"] == subprocess.DEVNULL
    assert "-i" not in captured["argv"] and "-t" not in captured["argv"]


def test_orthophoto_footprint_maps_valid_extent_to_world(tmp_path):
    """Real-flow footprint extraction from a geo-tagged RGBA orthophoto.

    100x80 px, 1 world-unit/px, tiepoint maps pixel (0,0) -> world (0, 80)
    (top-left; world Y decreases downward). Valid (alpha>0) rectangle is
    cols 10..49, rows 20..59, so the expected world extent is:
      x: [10, 50]  (10..49 -> outer edge 50)
      y: [20, 60]  (top row 20 -> world 60; bottom row 59 outer edge -> 20)
    """
    products_dir = tmp_path / "products"
    products_dir.mkdir(parents=True)
    _write_geotiff_orthophoto(
        products_dir / odm_runner.ORTHOPHOTO_FILENAME,
        width=100,
        height=80,
        scale_x=1.0,
        scale_y=1.0,
        world_x0=0.0,
        world_y0=80.0,
        valid_box=(10, 20, 49, 59),
    )

    geom = odm_runner.orthophoto_footprint(tmp_path)

    minx, miny, maxx, maxy = geom.bounds
    assert minx == pytest.approx(10.0)
    assert maxx == pytest.approx(50.0)
    assert miny == pytest.approx(20.0)
    assert maxy == pytest.approx(60.0)
    assert geom.area == pytest.approx(40.0 * 40.0)


def test_orthophoto_footprint_respects_pixel_scale_and_origin(tmp_path):
    """A non-unit pixel scale and non-zero world origin are applied correctly."""
    products_dir = tmp_path / "products"
    products_dir.mkdir(parents=True)
    # 2 world-units/px in X, 3 in Y; origin at world (1000, 5000).
    _write_geotiff_orthophoto(
        products_dir / odm_runner.ORTHOPHOTO_FILENAME,
        width=50,
        height=40,
        scale_x=2.0,
        scale_y=3.0,
        world_x0=1000.0,
        world_y0=5000.0,
        valid_box=(5, 10, 24, 19),
    )

    geom = odm_runner.orthophoto_footprint(tmp_path)

    minx, miny, maxx, maxy = geom.bounds
    # X: cols 5..24 -> [1000 + 5*2, 1000 + 25*2] = [1010, 1050]
    assert minx == pytest.approx(1010.0)
    assert maxx == pytest.approx(1050.0)
    # Y: rows 10..19 -> top 5000 - 10*3 = 4970; bottom 5000 - 20*3 = 4940
    assert miny == pytest.approx(4940.0)
    assert maxy == pytest.approx(4970.0)


def test_orthophoto_footprint_raises_without_geo_tags(tmp_path):
    products_dir = tmp_path / "products"
    products_dir.mkdir(parents=True)
    # A plain RGBA TIFF with no GeoTIFF tags.
    Image.new("RGBA", (10, 10), (0, 0, 0, 255)).save(
        products_dir / odm_runner.ORTHOPHOTO_FILENAME, format="TIFF"
    )
    with pytest.raises(odm_runner.OdmRunnerError):
        odm_runner.orthophoto_footprint(tmp_path)


def test_reconstruction_footprint_falls_back_to_orthophoto_without_sidecar(tmp_path):
    """No sidecar (the REAL ODM run case) -> footprint comes from the orthophoto."""
    products_dir = tmp_path / "products"
    products_dir.mkdir(parents=True)
    _write_geotiff_orthophoto(
        products_dir / odm_runner.ORTHOPHOTO_FILENAME,
        width=100,
        height=80,
        scale_x=1.0,
        scale_y=1.0,
        world_x0=0.0,
        world_y0=80.0,
        valid_box=(0, 0, 99, 79),
    )

    # No reconstruction_footprint.txt sidecar written.
    assert not (products_dir / odm_runner.FOOTPRINT_SIDECAR_FILENAME).exists()

    geom = odm_runner.reconstruction_footprint(tmp_path)
    assert geom.area == pytest.approx(100.0 * 80.0)
