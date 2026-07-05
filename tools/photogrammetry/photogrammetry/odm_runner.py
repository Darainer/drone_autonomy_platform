"""OpenDroneMap (ODM) container invocation + product collection.

# Implements: MAP-4

DES-005 D1/D6: reconstruction runs inside a pinned, published `opendronemap/odm`
container image (amd64 + arm64), invoked as an unattended `docker run` (STK-1(d)
-- no interactive flags, stdin closed, no prompts). Georeferencing (D6) uses an
ODM `geo.txt` file generated from the DES-004 `poses.csv` GNSS columns
(lat/lon/alt_amsl), written next to the dataset's `images/` directory per the
ODM georeferencing convention:

    <image_name> <geo_x=longitude> <geo_y=latitude> <geo_z=altitude>

Rows whose lat/lon are None (DES-004 D4 / TS-05 allows some GNSS-less frames)
are skipped -- ODM tolerates a geo.txt that only covers a subset of images,
falling back to relative/EXIF positioning for the rest.

Poses/manifest are consumed ONLY via `photogrammetry.dataset` (FE-1 forward
compatibility) -- no positional CSV parsing here.

# Injectable runner (test seam)
`run_odm()` takes a `runner` callable (`argv: List[str] -> CompletedProcess`)
so tests can substitute a fake ODM that writes stub product files without
Docker (Docker is unavailable in this environment and in CI). The default
runner (`_default_runner`) shells out to `docker` with `stdin=subprocess.DEVNULL`
and no `-i`/`-t`/`-it` flags -- this is asserted directly by
`build_odm_command()` unit tests.
"""
from __future__ import annotations

import dataclasses
import pathlib
import subprocess
from typing import Any, Callable, Dict, List, Optional

import numpy as np
from PIL import Image
from shapely.geometry import MultiPoint, box
from shapely.geometry.base import BaseGeometry

from photogrammetry.dataset import PoseRow

# GeoTIFF georeferencing tags (OGC GeoTIFF spec) used to map orthophoto
# pixel coordinates to world coordinates:
#   33550 ModelPixelScale  -> (scale_x, scale_y, scale_z)
#   33922 ModelTiepoint    -> (i, j, k, world_x, world_y, world_z), i.e.
#          raster point (i, j) maps to world (world_x, world_y). ODM writes
#          the top-left tiepoint (i=j=0), so raster rows increase *downward*
#          while world Y increases upward -> the Y scale is subtracted.
GEOTIFF_MODEL_PIXEL_SCALE_TAG = 33550
GEOTIFF_MODEL_TIEPOINT_TAG = 33922

# ODM version pin (DES-005 D1). Chosen at T3.3 implementation time; any
# *engine* version change should be reviewed the same way an engine change
# would be (DES-005 "Open questions" note) -- bump this constant and the
# matching pin in tools/photogrammetry/Dockerfile together.
ODM_IMAGE = "opendronemap/odm:3.5.4"

GEO_TXT_FILENAME = "geo.txt"
PRODUCTS_DIRNAME = "products"

# D6 product filenames, relative to <dataset>/products/.
LAZ_FILENAME = "odm_georeferenced_model.laz"
OBJ_FILENAME = "odm_textured_model_geo.obj"
MTL_FILENAME = "odm_textured_model_geo.mtl"
ORTHOPHOTO_FILENAME = "orthophoto.tif"

# Sidecar written by the (real or fake) ODM runner recording the ground
# footprint of the reconstruction, one "x y" vertex per line (the convex
# hull of the georeferenced point cloud's XY extent). This keeps
# `reconstruction_footprint()` testable without parsing binary .laz/.tif
# product formats: any runner (real ODM wrapper step, or a fake test
# runner) that produces the D6 products is also responsible for writing
# this sidecar next to them. See `reconstruction_footprint()` docstring.
FOOTPRINT_SIDECAR_FILENAME = "reconstruction_footprint.txt"


class OdmRunnerError(Exception):
    """Raised when ODM invocation or product collection fails."""


@dataclasses.dataclass
class OdmRunResult:
    argv: List[str]
    completed: subprocess.CompletedProcess
    products: Dict[str, pathlib.Path]


def write_geo_txt(dataset_dir, poses: List[PoseRow]) -> pathlib.Path:
    """Generate an ODM `geo.txt` from poses.csv GNSS columns (DES-005 D6).

    ODM's geo.txt format: a first line naming the coordinate reference
    system (`WGS84` for plain lat/lon/alt), followed by one line per image:

        <image_name> <geo_x=longitude> <geo_y=latitude> <geo_z=altitude>

    Rows with lat or lon == None (DES-004 D4 GNSS-optional columns) are
    skipped -- ODM only requires geo.txt to cover the images it can
    georeference directly; the rest fall back to ODM's own handling.
    Raises OdmRunnerError if not a single pose has usable lat/lon (nothing
    to georeference).
    """
    dataset_dir = pathlib.Path(dataset_dir)
    geo_txt_path = dataset_dir / GEO_TXT_FILENAME

    lines = ["WGS84"]
    for pose in poses:
        if pose.lat is None or pose.lon is None:
            continue
        alt = pose.alt_amsl if pose.alt_amsl is not None else 0.0
        lines.append(f"{pose.image_filename} {pose.lon:.9f} {pose.lat:.9f} {alt:.3f}")

    if len(lines) == 1:
        raise OdmRunnerError(
            "no pose has usable lat/lon GNSS columns -- cannot generate geo.txt for "
            "georeferenced ODM reconstruction"
        )

    with open(geo_txt_path, "w") as f:
        f.write("\n".join(lines) + "\n")

    return geo_txt_path


def build_odm_command(
    dataset_dir,
    geo_txt_path,
    image_scale: Optional[int] = None,
    extra_odm_args: Optional[List[str]] = None,
) -> List[str]:
    """Build the `docker run` argv for an unattended ODM reconstruction (STK-1(d)).

    Pure function -- no I/O, no subprocess invocation -- so it is directly
    unit-testable. Mounts `dataset_dir` at `/datasets/code` (ODM's expected
    project-folder convention: `<project>/images`, `<project>/odm_georeferencing/geo.txt`
    style outputs land under `<project>/`), points ODM at the generated
    geo.txt for georeferencing, and runs the container non-interactively:
    no `-i`/`-t`/`-it` flag anywhere in the returned argv (STK-1(d) -- the
    caller separately passes `stdin=subprocess.DEVNULL` when invoking this,
    but the argv itself must not request a tty/stdin attachment either).
    """
    dataset_dir = pathlib.Path(dataset_dir)
    geo_txt_path = pathlib.Path(geo_txt_path)

    argv = [
        "docker",
        "run",
        "--rm",
        "-v",
        f"{dataset_dir}:/datasets/code",
        ODM_IMAGE,
        "--project-path",
        "/datasets",
        "--geo",
        f"/datasets/code/{geo_txt_path.name}",
    ]
    if image_scale is not None:
        argv += ["--resize-to", str(image_scale)]
    if extra_odm_args:
        argv += list(extra_odm_args)
    return argv


def _default_runner(argv: List[str]) -> subprocess.CompletedProcess:
    """Real ODM invocation: unattended subprocess, stdin closed, no tty flags.

    STK-1(d): `stdin=subprocess.DEVNULL` so the container can never block on
    (or receive) interactive input, and `build_odm_command()` never emits
    `-i`/`-t`/`-it`. Never calls `input()`; never writes a prompt.
    """
    return subprocess.run(
        argv,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )


RunnerFn = Callable[[List[str]], subprocess.CompletedProcess]


def run_odm(
    dataset_dir,
    manifest: Dict[str, Any],
    poses: Optional[List[PoseRow]] = None,
    runner: Optional[RunnerFn] = None,
    image_scale: Optional[int] = None,
    extra_odm_args: Optional[List[str]] = None,
) -> OdmRunResult:
    """Orchestrate one ODM reconstruction run (DES-005 D1/D6).

    (1) generate geo.txt from poses' GNSS columns, (2) build the
    non-interactive `docker run` argv, (3) invoke it via `runner` (defaults
    to the real subprocess runner; tests inject a fake that writes stub
    products instead of running Docker), (4) collect D6 products.

    Raises OdmRunnerError if the runner reports a non-zero exit code or the
    expected products are not all present afterward.
    """
    from photogrammetry import dataset as ds  # local import avoids a cycle at module load

    dataset_dir = pathlib.Path(dataset_dir)
    if poses is None:
        poses = ds.read_poses(dataset_dir)

    geo_txt_path = write_geo_txt(dataset_dir, poses)
    argv = build_odm_command(dataset_dir, geo_txt_path, image_scale=image_scale, extra_odm_args=extra_odm_args)

    run = runner if runner is not None else _default_runner
    completed = run(argv)

    if completed.returncode != 0:
        raise OdmRunnerError(
            f"ODM container exited with code {completed.returncode}: "
            f"stdout={getattr(completed, 'stdout', '')!r} stderr={getattr(completed, 'stderr', '')!r}"
        )

    if not products_complete(dataset_dir):
        raise OdmRunnerError(
            f"ODM run completed (exit 0) but expected products are missing/empty under "
            f"{dataset_dir / PRODUCTS_DIRNAME}"
        )

    return OdmRunResult(argv=argv, completed=completed, products=collect_products(dataset_dir))


def collect_products(dataset_dir) -> Dict[str, pathlib.Path]:
    """Return the D6 product paths under `<dataset>/products/` (present or not)."""
    products_dir = pathlib.Path(dataset_dir) / PRODUCTS_DIRNAME
    return {
        "laz": products_dir / LAZ_FILENAME,
        "obj": products_dir / OBJ_FILENAME,
        "mtl": products_dir / MTL_FILENAME,
        "orthophoto": products_dir / ORTHOPHOTO_FILENAME,
    }


def products_complete(dataset_dir) -> bool:
    """True iff the .laz/.obj(+.mtl)/.tif D6 products all exist and are non-empty."""
    products = collect_products(dataset_dir)
    required = (products["laz"], products["obj"], products["mtl"], products["orthophoto"])
    return all(p.is_file() and p.stat().st_size > 0 for p in required)


def orthophoto_footprint(dataset_dir) -> BaseGeometry:
    """Derive the reconstruction footprint from the REAL orthophoto product (DES-005 D4).

    DES-005 D4 explicitly allows an "orthophoto ... footprint polygon" as the
    coverage-QA input. This reads `products/orthophoto.tif` (a georeferenced
    RGBA GeoTIFF, as ODM emits) with Pillow -- already a dependency, so no
    heavy laspy/PDAL/GDAL install is needed on the Jetson target -- and:

      (1) reads the GeoTIFF geo-transform tags ModelPixelScale (33550) and
          ModelTiepoint (33922) to map pixel (col, row) -> world (x, y);
      (2) uses the alpha channel (valid data = alpha > 0) to bound the
          VALID-data extent (ODM orthophotos are nodata-transparent outside
          the reconstructed area), then returns that valid-data bounding box
          as a world-frame polygon.

    Returns the footprint in the same world frame the manifest survey polygon
    is expressed in. Raises OdmRunnerError if the orthophoto is missing,
    lacks the geo-transform tags, or has no valid (alpha>0) pixels.

    This is the REAL-flow footprint source used when no sidecar cache is
    present (see `reconstruction_footprint`), so a genuine ODM run yields a
    real coverage verdict rather than depending on a test-only sidecar.
    """
    dataset_dir = pathlib.Path(dataset_dir)
    ortho_path = dataset_dir / PRODUCTS_DIRNAME / ORTHOPHOTO_FILENAME
    if not ortho_path.is_file():
        raise OdmRunnerError(f"orthophoto not found: {ortho_path}")

    with Image.open(ortho_path) as img:
        tags = getattr(img, "tag_v2", None)
        if tags is None or GEOTIFF_MODEL_PIXEL_SCALE_TAG not in tags or GEOTIFF_MODEL_TIEPOINT_TAG not in tags:
            raise OdmRunnerError(
                f"{ortho_path} lacks GeoTIFF geo-transform tags "
                f"(ModelPixelScale/{GEOTIFF_MODEL_PIXEL_SCALE_TAG} and "
                f"ModelTiepoint/{GEOTIFF_MODEL_TIEPOINT_TAG}); cannot georeference the footprint"
            )
        scale = tags[GEOTIFF_MODEL_PIXEL_SCALE_TAG]
        tiepoint = tags[GEOTIFF_MODEL_TIEPOINT_TAG]
        scale_x = float(scale[0])
        scale_y = float(scale[1])
        raster_i = float(tiepoint[0])
        raster_j = float(tiepoint[1])
        world_x0 = float(tiepoint[3])
        world_y0 = float(tiepoint[4])

        arr = np.asarray(img)

    if arr.ndim != 3 or arr.shape[2] < 4:
        raise OdmRunnerError(
            f"{ortho_path} is not an RGBA orthophoto (shape {getattr(arr, 'shape', None)}); "
            f"cannot determine the valid-data (alpha) mask"
        )

    alpha = arr[:, :, 3]
    valid_rows, valid_cols = np.where(alpha > 0)
    if valid_rows.size == 0:
        raise OdmRunnerError(f"{ortho_path} has no valid (alpha>0) pixels")

    min_col = float(valid_cols.min())
    max_col = float(valid_cols.max())
    min_row = float(valid_rows.min())
    max_row = float(valid_rows.max())

    # Map the valid-pixel-cell corners to world coordinates. The extent spans
    # the OUTER edges of the boundary pixels, so use max_col+1 / max_row+1.
    def _to_world(col: float, row: float):
        wx = world_x0 + (col - raster_i) * scale_x
        wy = world_y0 - (row - raster_j) * scale_y  # raster rows grow downward
        return wx, wy

    x_left, y_top = _to_world(min_col, min_row)
    x_right, y_bottom = _to_world(max_col + 1.0, max_row + 1.0)

    minx, maxx = min(x_left, x_right), max(x_left, x_right)
    miny, maxy = min(y_top, y_bottom), max(y_top, y_bottom)
    return box(minx, miny, maxx, maxy)


def reconstruction_footprint(dataset_dir) -> BaseGeometry:
    """Derive the reconstruction's ground-coverage footprint (DES-005 D4 input).

    Resolution order:
      1. If the optional `products/reconstruction_footprint.txt` sidecar is
         present (one "x y" vertex per line), use its convex hull. This is an
         optional override/cache: a real wrapper may pre-populate it from the
         .laz point-cloud hull at product-collection time, and the fake ODM
         runner in the TS-08 smoke test writes it to exercise this path
         without a real GeoTIFF.
      2. Otherwise (the default on a genuine ODM run, which does NOT write the
         sidecar), fall back to `orthophoto_footprint()` -- derived directly
         from the real `products/orthophoto.tif` GeoTIFF, per DES-005 D4.

    Raises OdmRunnerError if neither source yields a usable footprint (sidecar
    malformed / < 3 vertices, or orthophoto missing/un-georeferenced/blank).
    """
    dataset_dir = pathlib.Path(dataset_dir)
    sidecar_path = dataset_dir / PRODUCTS_DIRNAME / FOOTPRINT_SIDECAR_FILENAME
    if not sidecar_path.is_file():
        # Real ODM run: no sidecar -> derive the footprint from the actual
        # orthophoto product so coverage QA is genuine, not hollow.
        return orthophoto_footprint(dataset_dir)

    vertices = []
    with open(sidecar_path, "r") as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 2:
                raise OdmRunnerError(f"{sidecar_path} line {line_no} malformed: {line!r}")
            vertices.append((float(parts[0]), float(parts[1])))

    if len(vertices) < 3:
        raise OdmRunnerError(
            f"{sidecar_path} has fewer than 3 vertices ({len(vertices)}) -- cannot derive a "
            f"reconstruction footprint polygon"
        )

    return MultiPoint(vertices).convex_hull


def write_footprint_sidecar(dataset_dir, vertices) -> pathlib.Path:
    """Write the `reconstruction_footprint.txt` sidecar consumed by `reconstruction_footprint()`.

    Convenience for runners (real product-collection step, or a fake test
    runner) that already have the point-cloud XY extent as a list of
    (x, y) vertices.
    """
    products_dir = pathlib.Path(dataset_dir) / PRODUCTS_DIRNAME
    products_dir.mkdir(parents=True, exist_ok=True)
    sidecar_path = products_dir / FOOTPRINT_SIDECAR_FILENAME
    with open(sidecar_path, "w") as f:
        for x, y in vertices:
            f.write(f"{x} {y}\n")
    return sidecar_path
