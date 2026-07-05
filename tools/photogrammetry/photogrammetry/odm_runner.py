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
import shutil
import subprocess
from typing import Any, Callable, Dict, List, Optional

import numpy as np
from PIL import Image
from shapely.affinity import affine_transform
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

# GeoTIFF GeoKeyDirectory (tag 34735) + the CRS GeoKeys we read from it to
# decide whether the orthophoto's world coordinates are geographic (lon/lat)
# or projected. GTModelTypeGeoKey values: 1 = Projected, 2 = Geographic,
# 3 = Geocentric. GeographicTypeGeoKey 4326 = WGS84. See OGC GeoTIFF spec.
GEOTIFF_GEOKEY_DIRECTORY_TAG = 34735
GEOKEY_GT_MODEL_TYPE = 1024
GEOKEY_GEOGRAPHIC_TYPE = 2048
GEOKEY_PROJECTED_CS_TYPE = 3072
GT_MODEL_TYPE_PROJECTED = 1
GT_MODEL_TYPE_GEOGRAPHIC = 2

# ODM version pin (DES-005 D1). Chosen at T3.3 implementation time; any
# *engine* version change should be reviewed the same way an engine change
# would be (DES-005 "Open questions" note) -- bump this constant and the
# matching pin in tools/photogrammetry/Dockerfile together.
ODM_IMAGE = "opendronemap/odm:3.5.4"

GEO_TXT_FILENAME = "geo.txt"
PRODUCTS_DIRNAME = "products"

# ODM docker mount/name scheme. The dataset dir is mounted as the ODM
# *project* folder `/datasets/code`; ODM's positional dataset-name argument
# is therefore `code`, and it expects images under `<name>/images` (already
# satisfied: <dataset>/images -> /datasets/code/images) and reads geo.txt at
# /datasets/code/geo.txt. `--project-path /datasets` + positional `code`.
ODM_PROJECT_PATH = "/datasets"
ODM_DATASET_NAME = "code"
ODM_PROJECT_MOUNT = f"{ODM_PROJECT_PATH}/{ODM_DATASET_NAME}"  # /datasets/code

# D6 product filenames, relative to <dataset>/products/ (the canonical D6
# tree the rest of the pipeline reads).
LAZ_FILENAME = "odm_georeferenced_model.laz"
OBJ_FILENAME = "odm_textured_model_geo.obj"
MTL_FILENAME = "odm_textured_model_geo.mtl"
ORTHOPHOTO_FILENAME = "orthophoto.tif"

# Real ODM does NOT write to <dataset>/products/; with the standard docker
# invocation it writes into per-stage subfolders under the project dataset
# dir (mounted at /datasets/code -> <dataset>/). run_odm() copies/normalizes
# these into <dataset>/products/ with the D6 names above (see
# collect_odm_outputs). Source paths are relative to <dataset>/.
ODM_OUTPUT_SOURCES = (
    ("odm_georeferencing/odm_georeferenced_model.laz", LAZ_FILENAME),
    ("odm_texturing/odm_textured_model_geo.obj", OBJ_FILENAME),
    ("odm_texturing/odm_textured_model_geo.mtl", MTL_FILENAME),
    ("odm_orthophoto/odm_orthophoto.tif", ORTHOPHOTO_FILENAME),
)
# The textured mesh's material references texture image files; copy them
# alongside the .obj/.mtl so the mesh renders from <dataset>/products/.
ODM_TEXTURE_SUBFOLDER = "odm_texturing"
ODM_TEXTURE_GLOB = "*.png"

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
    unit-testable. Mounts `dataset_dir` at `/datasets/code` (the ODM project
    folder), passes ODM's POSITIONAL dataset-name argument (`code`) alongside
    `--project-path /datasets`, and points ODM at the generated geo.txt for
    georeferencing. ODM finds images at `<project>/<name>/images` (i.e.
    /datasets/code/images -> <dataset>/images) and writes its per-stage
    outputs under the project dir (collected afterward by collect_odm_outputs).

    Runs the container non-interactively: no `-i`/`-t`/`-it` flag anywhere in
    the returned argv (STK-1(d) -- the caller separately passes
    `stdin=subprocess.DEVNULL` when invoking this, but the argv itself must
    not request a tty/stdin attachment either).
    """
    dataset_dir = pathlib.Path(dataset_dir)
    geo_txt_path = pathlib.Path(geo_txt_path)

    argv = [
        "docker",
        "run",
        "--rm",
        "-v",
        f"{dataset_dir}:{ODM_PROJECT_MOUNT}",
        ODM_IMAGE,
        "--project-path",
        ODM_PROJECT_PATH,
        ODM_DATASET_NAME,  # ODM positional dataset name
        "--geo",
        f"{ODM_PROJECT_MOUNT}/{geo_txt_path.name}",
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
    to the real subprocess runner; tests inject a fake that writes ODM's real
    output subfolders instead of running Docker), (4) collect/normalize ODM's
    per-stage outputs into `<dataset>/products/` with the D6 names.

    Raises OdmRunnerError if the runner reports a non-zero exit code or the
    expected products are not all present (and non-empty) after collection.
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

    # ODM wrote into its per-stage subfolders under <dataset>/; normalize
    # those into the canonical <dataset>/products/ D6 tree BEFORE checking
    # completeness (products_complete must pass only after collection).
    collect_odm_outputs(dataset_dir)

    if not products_complete(dataset_dir):
        raise OdmRunnerError(
            f"ODM run completed (exit 0) but expected products are missing/empty under "
            f"{dataset_dir / PRODUCTS_DIRNAME} after collecting ODM outputs"
        )

    return OdmRunResult(argv=argv, completed=completed, products=collect_products(dataset_dir))


def collect_odm_outputs(dataset_dir) -> Dict[str, pathlib.Path]:
    """Copy/normalize ODM's per-stage outputs into `<dataset>/products/` (D6 names).

    Real ODM writes `odm_georeferencing/odm_georeferenced_model.laz`,
    `odm_texturing/odm_textured_model_geo.obj` (+ `.mtl` + texture images),
    and `odm_orthophoto/odm_orthophoto.tif` under the project dir. This copies
    each into `<dataset>/products/` with the D6 filenames the rest of the
    pipeline reads, plus any texture image files the mesh material references.
    Missing sources are skipped here -- `products_complete()` is the gate that
    reports an incomplete reconstruction. Returns the collected D6 paths.
    """
    dataset_dir = pathlib.Path(dataset_dir)
    products_dir = dataset_dir / PRODUCTS_DIRNAME
    products_dir.mkdir(parents=True, exist_ok=True)

    for src_rel, dst_name in ODM_OUTPUT_SOURCES:
        src = dataset_dir / src_rel
        if src.is_file():
            shutil.copy2(src, products_dir / dst_name)

    # Texture images referenced by the .mtl (best-effort).
    texture_dir = dataset_dir / ODM_TEXTURE_SUBFOLDER
    if texture_dir.is_dir():
        for tex in texture_dir.glob(ODM_TEXTURE_GLOB):
            if tex.is_file():
                shutil.copy2(tex, products_dir / tex.name)

    return collect_products(dataset_dir)


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


def _read_geokeys(tags) -> Dict[int, int]:
    """Parse the GeoTIFF GeoKeyDirectory (tag 34735) into {key_id: value}.

    Only in-directory values (TIFFTagLocation == 0) are returned -- that
    covers the CRS-type keys we need (GTModelTypeGeoKey, GeographicTypeGeoKey,
    ProjectedCSTypeGeoKey). Returns {} if the directory is absent/malformed.
    """
    raw = None if tags is None else tags.get(GEOTIFF_GEOKEY_DIRECTORY_TAG)
    if not raw:
        return {}
    vals = [int(v) for v in raw]
    if len(vals) < 4:
        return {}
    num_keys = vals[3]
    keys: Dict[int, int] = {}
    for k in range(num_keys):
        off = 4 + k * 4
        if off + 3 >= len(vals):
            break
        key_id, tag_loc, _count, value_off = vals[off : off + 4]
        if tag_loc == 0:  # value stored directly in the directory entry
            keys[key_id] = value_off
    return keys


def orthophoto_footprint(dataset_dir) -> BaseGeometry:
    """Reconstruction footprint from the REAL orthophoto, in GEOGRAPHIC lon/lat (DES-005 D4).

    DES-005 D4 allows an "orthophoto ... footprint polygon" as the coverage-QA
    input. This reads `products/orthophoto.tif` (a georeferenced RGBA GeoTIFF,
    as ODM emits) with Pillow -- no heavy laspy/PDAL/GDAL install needed on the
    Jetson target -- and:

      (1) reads the CRS GeoKeys (tag 34735). ODM is driven by a WGS84 lon/lat
          geo.txt (see write_geo_txt), so v1 supports a GEOGRAPHIC (lon/lat)
          orthophoto CRS, in which the ModelTiepoint/PixelScale world coords
          ARE lon/lat. A PROJECTED CRS (e.g. UTM) raises a clear, actionable
          error rather than silently comparing mismatched frames downstream
          (v1 limitation -- validate/extend at the TS-08 Docker / TS-11 HIL
          gate, where real ODM output CRS is confirmed);
      (2) reads ModelPixelScale (33550) + ModelTiepoint (33922) to map pixel
          (col, row) -> world (lon, lat);
      (3) uses the alpha channel (valid data = alpha > 0) to bound the
          VALID-data extent (ODM orthophotos are nodata-transparent outside
          the reconstructed area), returned as the valid-data bounding box.

    Returns the footprint in GEOGRAPHIC (lon, lat) coordinates. The caller
    (reconstruction_footprint) reconciles this into the local ENU frame the
    manifest survey polygon uses (DES-004 D3). Raises OdmRunnerError if the
    orthophoto is missing, lacks geo tags, has a projected/unknown CRS, is not
    RGBA, or has no valid (alpha>0) pixels.
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

        geokeys = _read_geokeys(tags)
        model_type = geokeys.get(GEOKEY_GT_MODEL_TYPE)
        if model_type == GT_MODEL_TYPE_PROJECTED:
            proj_cs = geokeys.get(GEOKEY_PROJECTED_CS_TYPE)
            raise OdmRunnerError(
                f"projected orthophoto CRS {proj_cs} not yet supported for onboard ENU "
                f"reconciliation; validate/extend at the TS-08 Docker / TS-11 HIL gate"
            )
        if model_type != GT_MODEL_TYPE_GEOGRAPHIC:
            raise OdmRunnerError(
                f"{ortho_path} CRS GeoKeys missing/unknown (GTModelTypeGeoKey={model_type}); "
                f"expected a geographic (lon/lat, GTModelTypeGeoKey={GT_MODEL_TYPE_GEOGRAPHIC}) "
                f"orthophoto in v1"
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

    # Map the valid-pixel-cell corners to world (lon, lat). The extent spans
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


def lonlat_to_enu_fit(poses: List[PoseRow]) -> np.ndarray:
    """Least-squares 2D affine mapping geographic (lon, lat) -> local ENU (x, y).

    DES-004 D3: coverage QA is done in the local ENU frame; poses.csv records
    BOTH local ENU x/y AND lat/lon per frame precisely so the transform can be
    fit. Over a survey-sized area a linear/affine tangent-plane fit is accurate
    (pure numpy, no pyproj). Uses only poses with BOTH non-null lat/lon AND
    (finite) ENU x/y.

    Returns a 2x3 matrix A such that [x, y]^T = A @ [lon, lat, 1]^T. Raises
    OdmRunnerError if fewer than 3 usable poses, or if the (lon, lat) samples
    are collinear (an affine fit is then underdetermined).
    """
    samples = [
        (p.lon, p.lat, p.x, p.y)
        for p in poses
        if p.lon is not None and p.lat is not None
    ]
    if len(samples) < 3:
        raise OdmRunnerError(
            f"need >= 3 poses with both GNSS lon/lat and ENU x/y to fit the geographic->ENU "
            f"affine; got {len(samples)}"
        )

    arr = np.asarray(samples, dtype=float)
    lon, lat, x, y = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]
    design = np.column_stack([lon, lat, np.ones(len(arr))])
    if np.linalg.matrix_rank(design) < 3:
        raise OdmRunnerError(
            "GNSS (lon, lat) sample points are collinear; cannot fit a 2D affine "
            "geographic->ENU transform (need >= 3 non-collinear fixes)"
        )

    coef_x, _res_x, _rank_x, _sv_x = np.linalg.lstsq(design, x, rcond=None)
    coef_y, _res_y, _rank_y, _sv_y = np.linalg.lstsq(design, y, rcond=None)
    return np.vstack([coef_x, coef_y])  # 2x3


def _enu_affine_coeffs(A: np.ndarray) -> List[float]:
    """Convert a 2x3 [lon,lat,1]->[x,y] affine into shapely.affine_transform coeffs.

    shapely wants [a, b, d, e, xoff, yoff] with x' = a*x + b*y + xoff,
    y' = d*x + e*y + yoff (here x=lon, y=lat).
    """
    return [A[0, 0], A[0, 1], A[1, 0], A[1, 1], A[0, 2], A[1, 2]]


def reconstruction_footprint(dataset_dir, poses: Optional[List[PoseRow]] = None) -> BaseGeometry:
    """Reconstruction ground-coverage footprint in the LOCAL ENU frame (DES-005 D4 / DES-004 D3).

    Returned in the SAME local ENU frame as the manifest survey polygon so
    coverage QA compares like with like.

    Resolution order:
      1. If the optional `products/reconstruction_footprint.txt` sidecar is
         present (one "x y" vertex per line), use its convex hull directly --
         the sidecar is documented to carry ENU vertices. This is an
         override/cache a real wrapper may pre-populate from the .laz hull;
         the TS-08 smoke-test fake runner writes it to drive full mode without
         a real geographic GeoTIFF.
      2. Otherwise (the default on a genuine ODM run, which does NOT write the
         sidecar), take the orthophoto footprint in geographic lon/lat
         (orthophoto_footprint) and map it into ENU through the affine fit by
         lonlat_to_enu_fit(poses). `poses` is REQUIRED in this path.

    Raises OdmRunnerError if neither source yields a usable ENU footprint
    (sidecar malformed / < 3 vertices; or no poses / un-georeferenced /
    projected-CRS / blank orthophoto).
    """
    dataset_dir = pathlib.Path(dataset_dir)
    sidecar_path = dataset_dir / PRODUCTS_DIRNAME / FOOTPRINT_SIDECAR_FILENAME

    if sidecar_path.is_file():
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

    # Real ODM run: no sidecar. Derive from the orthophoto (geographic lon/lat)
    # and reconcile into ENU via the per-mission affine fit (DES-004 D3).
    if poses is None:
        raise OdmRunnerError(
            "poses are required to reconcile the orthophoto footprint into the local ENU frame "
            "(no reconstruction_footprint.txt sidecar present)"
        )
    geographic_footprint = orthophoto_footprint(dataset_dir)
    affine = lonlat_to_enu_fit(poses)
    return affine_transform(geographic_footprint, _enu_affine_coeffs(affine))


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
