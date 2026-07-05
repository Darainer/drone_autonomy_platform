"""Unit tests for photogrammetry/odm_runner.py: geo.txt generation + argv shape.

# Verifies: MAP-4
"""
from __future__ import annotations

import pathlib
import subprocess

import numpy as np
import pytest
from PIL import Image
from PIL.TiffImagePlugin import ImageFileDirectory_v2

from photogrammetry import dataset as ds, odm_runner
from photogrammetry.dataset import PoseRow


def _geokey_directory(crs):
    """Build a GeoKeyDirectory (tag 34735) SHORT array for a given CRS kind.

    'geographic' -> WGS84 lon/lat (GTModelTypeGeoKey=2, GeographicTypeGeoKey=4326)
    'projected'  -> UTM 10N       (GTModelTypeGeoKey=1, ProjectedCSTypeGeoKey=32610)
    None         -> no directory (caller omits the tag).
    """
    if crs == "geographic":
        return (1, 1, 0, 2, 1024, 0, 1, 2, 2048, 0, 1, 4326)
    if crs == "projected":
        return (1, 1, 0, 2, 1024, 0, 1, 1, 3072, 0, 1, 32610)
    return None


def _write_geotiff_orthophoto(
    path,
    width,
    height,
    scale_x,
    scale_y,
    world_x0,
    world_y0,
    valid_box,
    crs="geographic",
):
    """Write a synthetic RGBA GeoTIFF with ModelPixelScale/ModelTiepoint + CRS GeoKeys.

    `valid_box` = (min_col, min_row, max_col, max_row) inclusive pixel range
    that is marked opaque (alpha=255); everything else is transparent
    (alpha=0), mimicking an ODM nodata-transparent orthophoto. The tiepoint
    maps raster (0, 0) -> world (world_x0, world_y0) (top-left). `crs` controls
    the GeoKeyDirectory (geographic WGS84 by default).
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

    geokeys = _geokey_directory(crs)
    if geokeys is not None:
        info.tagtype[odm_runner.GEOTIFF_GEOKEY_DIRECTORY_TAG] = 3  # SHORT
        info[odm_runner.GEOTIFF_GEOKEY_DIRECTORY_TAG] = geokeys

    img.save(path, format="TIFF", tiffinfo=info)


def _pose(frame_idx, lon, lat, x, y):
    """Minimal PoseRow carrying the lon/lat + ENU x/y needed for the affine fit."""
    return PoseRow(
        frame_idx=frame_idx,
        stamp_ns=1_700_000_000_000_000_000 + frame_idx,
        x=x,
        y=y,
        z=50.0,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
        pose_stamp_ns=0,
        lat=lat,
        lon=lon,
        alt_amsl=100.0,
        gnss_stamp_ns=0,
        sync_err_ms=5.0,
    )


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


def test_build_odm_command_has_positional_dataset_name_and_mount():
    """ODM needs its POSITIONAL dataset-name arg + a matching project mount."""
    dataset_dir = pathlib.Path("/some/dataset")
    geo_txt_path = dataset_dir / "geo.txt"

    argv = odm_runner.build_odm_command(dataset_dir, geo_txt_path)

    # Positional dataset name present, and --project-path set.
    assert odm_runner.ODM_DATASET_NAME in argv
    assert "--project-path" in argv
    proj_idx = argv.index("--project-path")
    assert argv[proj_idx + 1] == odm_runner.ODM_PROJECT_PATH
    # The dataset dir is mounted at the ODM project mount, and geo.txt/images
    # are addressed under that mount.
    assert f"{dataset_dir}:{odm_runner.ODM_PROJECT_MOUNT}" in argv
    geo_idx = argv.index("--geo")
    assert argv[geo_idx + 1] == f"{odm_runner.ODM_PROJECT_MOUNT}/geo.txt"
    # Still non-interactive.
    assert not ({"-i", "-t", "-it", "-ti"} & set(argv))


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

    # Sidecar carries ENU vertices directly (no poses needed).
    geom = odm_runner.reconstruction_footprint(dataset_dir)
    assert geom.area == pytest.approx(100.0 * 80.0)


def test_reconstruction_footprint_raises_when_no_sidecar_and_no_poses(tmp_path):
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


def test_orthophoto_footprint_raises_on_projected_crs(tmp_path):
    """A PROJECTED (UTM) orthophoto CRS must fail loudly, not silently mis-compare frames."""
    products_dir = tmp_path / "products"
    products_dir.mkdir(parents=True)
    _write_geotiff_orthophoto(
        products_dir / odm_runner.ORTHOPHOTO_FILENAME,
        width=50,
        height=40,
        scale_x=1.0,
        scale_y=1.0,
        world_x0=500000.0,
        world_y0=4000000.0,
        valid_box=(0, 0, 49, 39),
        crs="projected",
    )

    with pytest.raises(odm_runner.OdmRunnerError, match="projected"):
        odm_runner.orthophoto_footprint(tmp_path)


# ---------------------------------------------------------------------------
# FIX 2 -- coverage QA reconciled into the LOCAL ENU frame (DES-004 D3).
# A KNOWN affine maps geographic (lon, lat) -> ENU (x, y):
#     x = 10000 * lon + 1224000      (lon in [-122.40, -122.39] -> x in [0, 100])
#     y = 8000  * lat - 301600       (lat in [ 37.70,   37.71] -> y in [0, 80])
# ---------------------------------------------------------------------------
_A_LON, _B_LON = 10000.0, 1224000.0
_A_LAT, _B_LAT = 8000.0, -301600.0


def _enu_from_lonlat(lon, lat):
    return (_A_LON * lon + _B_LON, _A_LAT * lat + _B_LAT)


# Four non-collinear GNSS corners spanning the survey area.
_FIT_POSES = [
    _pose(i, lon, lat, *_enu_from_lonlat(lon, lat))
    for i, (lon, lat) in enumerate(
        [(-122.40, 37.70), (-122.39, 37.70), (-122.40, 37.71), (-122.39, 37.71)]
    )
]


def test_lonlat_to_enu_fit_recovers_known_affine():
    A = odm_runner.lonlat_to_enu_fit(_FIT_POSES)
    # Round-trip a handful of geographic points through the fitted affine.
    for lon, lat in [(-122.40, 37.70), (-122.395, 37.705), (-122.39, 37.71)]:
        x = A[0, 0] * lon + A[0, 1] * lat + A[0, 2]
        y = A[1, 0] * lon + A[1, 1] * lat + A[1, 2]
        exp_x, exp_y = _enu_from_lonlat(lon, lat)
        assert x == pytest.approx(exp_x, abs=1e-4)
        assert y == pytest.approx(exp_y, abs=1e-4)


def test_lonlat_to_enu_fit_raises_on_collinear_gnss():
    # All samples on a single geographic line -> affine underdetermined.
    collinear = [_pose(i, -122.40 + i * 1e-4, 37.70 + i * 1e-4, float(i), float(i)) for i in range(5)]
    with pytest.raises(odm_runner.OdmRunnerError):
        odm_runner.lonlat_to_enu_fit(collinear)


def test_lonlat_to_enu_fit_raises_with_too_few_poses():
    with pytest.raises(odm_runner.OdmRunnerError):
        odm_runner.lonlat_to_enu_fit(_FIT_POSES[:2])


def test_reconstruction_footprint_reconciles_orthophoto_into_enu(tmp_path):
    """Real flow: geographic orthophoto -> ENU via the fitted affine, coverage in ENU.

    The ortho's valid extent is lon in [-122.40, -122.39], lat in [37.70, 37.71],
    which maps through the known affine to the ENU box [0,100] x [0,80] -- a
    superset of the ENU survey polygon, so coverage is ~100%.
    """
    products_dir = tmp_path / "products"
    products_dir.mkdir(parents=True)
    # 100x80 px covering 0.01 deg in each axis; top-left px -> (lon=-122.40, lat=37.71).
    _write_geotiff_orthophoto(
        products_dir / odm_runner.ORTHOPHOTO_FILENAME,
        width=100,
        height=80,
        scale_x=0.01 / 100.0,
        scale_y=0.01 / 80.0,
        world_x0=-122.40,
        world_y0=37.71,
        valid_box=(0, 0, 99, 79),
        crs="geographic",
    )
    assert not (products_dir / odm_runner.FOOTPRINT_SIDECAR_FILENAME).exists()

    enu_footprint = odm_runner.reconstruction_footprint(tmp_path, _FIT_POSES)

    minx, miny, maxx, maxy = enu_footprint.bounds
    assert minx == pytest.approx(0.0, abs=1e-3)
    assert maxx == pytest.approx(100.0, abs=1e-3)
    assert miny == pytest.approx(0.0, abs=1e-3)
    assert maxy == pytest.approx(80.0, abs=1e-3)

    # Coverage against an ENU survey polygon fully inside the footprint -> 100%.
    from photogrammetry import coverage
    from shapely.geometry import Polygon

    survey = Polygon([(0, 0), (100, 0), (100, 80), (0, 80)])
    result = coverage.compute_coverage(enu_footprint, survey, threshold=95.0)
    assert result["coverage_pct"] == pytest.approx(100.0, abs=1e-2)
    assert result["verdict"] == "pass"


def test_run_odm_collects_from_odm_output_subfolders(tmp_path, dataset_factory):
    """run_odm must normalize ODM's per-stage subfolder outputs into products/."""
    dataset_dir = dataset_factory(mission_id="collect_test")

    def fake_runner(argv):
        # Emulate real ODM writing into its per-stage output subfolders.
        (dataset_dir / "odm_georeferencing").mkdir(parents=True, exist_ok=True)
        (dataset_dir / "odm_georeferencing" / "odm_georeferenced_model.laz").write_bytes(b"laz")
        (dataset_dir / "odm_texturing").mkdir(parents=True, exist_ok=True)
        (dataset_dir / "odm_texturing" / "odm_textured_model_geo.obj").write_bytes(b"obj")
        (dataset_dir / "odm_texturing" / "odm_textured_model_geo.mtl").write_bytes(b"mtl")
        (dataset_dir / "odm_texturing" / "odm_textured_model_geo_material0000.png").write_bytes(b"png")
        (dataset_dir / "odm_orthophoto").mkdir(parents=True, exist_ok=True)
        (dataset_dir / "odm_orthophoto" / "odm_orthophoto.tif").write_bytes(b"tif")
        return subprocess.CompletedProcess(argv, returncode=0, stdout="", stderr="")

    # products/ must not exist before the run (proves collection populated it).
    assert not (dataset_dir / "products").exists()

    result = odm_runner.run_odm(dataset_dir, manifest={}, runner=fake_runner)

    assert odm_runner.products_complete(dataset_dir)
    products = odm_runner.collect_products(dataset_dir)
    for key in ("laz", "obj", "mtl", "orthophoto"):
        assert products[key].is_file() and products[key].stat().st_size > 0
    # Texture image copied alongside.
    assert (dataset_dir / "products" / "odm_textured_model_geo_material0000.png").is_file()
    assert result.completed.returncode == 0
