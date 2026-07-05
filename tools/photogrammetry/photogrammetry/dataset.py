"""DES-004 survey dataset reader + schema validation.

Dataset layout (see docs/design/DES-004-survey-dataset-recording.md):

    survey_<mission_id>/
      manifest.yaml     # format_version, mission_id, polygon, altitude,
                        # overlaps, start/end UTC, frame_count, camera
                        # intrinsics, sha256 per file
      images/
        <frame_idx:06d>_<stamp_ns>.jpg
      poses.csv         # frame_idx, stamp_ns, x, y, z, qx, qy, qz, qw,
                        # pose_stamp_ns, lat, lon, alt_amsl, gnss_stamp_ns,
                        # sync_err_ms

# FE-1 forward-compat contract (DES-004): RTK columns (fix type, covariance)
# and fused-pose columns will be added to poses.csv later, and manifest.yaml
# may grow additive keys; `format_version` covers migration. This reader
# loads manifest.yaml as a plain dict and reads poses.csv by column NAME via
# csv.DictReader (never by positional index/width). Only the REQUIRED
# keys/columns are checked for presence; any unknown extra key/column is
# passed through untouched and must never cause a validation failure.
"""
from __future__ import annotations

import csv
import dataclasses
import hashlib
import pathlib
from typing import Any, Dict, List, Optional

import yaml

MANIFEST_FILENAME = "manifest.yaml"
PARTIAL_MANIFEST_FILENAME = "manifest.yaml.part"
POSES_FILENAME = "poses.csv"
IMAGES_DIRNAME = "images"

# Required top-level manifest keys (DES-004). Unknown extra keys are
# ignored -- see FE-1 note above.
REQUIRED_MANIFEST_KEYS = (
    "format_version",
    "mission_id",
    "polygon",
    "altitude",
    "overlaps",
    "start_utc",
    "end_utc",
    "frame_count",
    "camera_intrinsics",
    "sha256",
)

REQUIRED_CAMERA_INTRINSICS_KEYS = ("fx", "fy", "cx", "cy", "width", "height")
REQUIRED_OVERLAP_KEYS = ("forward", "side")

# Required poses.csv columns (DES-004). Read by name via csv.DictReader;
# additional columns (RTK fix type/covariance, fused pose, ...) are ignored
# -- see FE-1 note above.
REQUIRED_POSE_COLUMNS = (
    "frame_idx",
    "stamp_ns",
    "x",
    "y",
    "z",
    "qx",
    "qy",
    "qz",
    "qw",
    "pose_stamp_ns",
    "lat",
    "lon",
    "alt_amsl",
    "gnss_stamp_ns",
    "sync_err_ms",
)

# The lowest format_version this reader understands. FE-1: a manifest with a
# *higher* format_version that still carries all required v1 fields must
# keep working (additive migration), so this is a floor, not an exact match.
MIN_SUPPORTED_FORMAT_VERSION = 1


class DatasetError(Exception):
    """Raised when a DES-004 dataset is missing, malformed, or fails schema validation."""


@dataclasses.dataclass
class PoseRow:
    frame_idx: int
    stamp_ns: int
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    pose_stamp_ns: int
    # GNSS columns are Optional: per DES-004 D4 the GNSS fix is attached
    # "most-recent-within-200 ms" and may be absent for some frames (TP-002
    # TS-05 allows up to 1% of rows with null lat/lon). An empty/missing cell
    # in any of these four parses to None rather than raising.
    lat: Optional[float]
    lon: Optional[float]
    alt_amsl: Optional[float]
    gnss_stamp_ns: Optional[int]
    sync_err_ms: float
    # Any additional (future/FE-1) columns present in the CSV, verbatim.
    extra: Dict[str, str] = dataclasses.field(default_factory=dict)

    @property
    def image_filename(self) -> str:
        return f"{self.frame_idx:06d}_{self.stamp_ns}.jpg"


@dataclasses.dataclass
class Dataset:
    root: pathlib.Path
    manifest: Dict[str, Any]
    poses: List[PoseRow]

    @property
    def images_dir(self) -> pathlib.Path:
        return self.root / IMAGES_DIRNAME

    @property
    def poses_path(self) -> pathlib.Path:
        return self.root / POSES_FILENAME

    @property
    def manifest_path(self) -> pathlib.Path:
        return self.root / MANIFEST_FILENAME


def find_manifest_path(dataset_dir) -> pathlib.Path:
    """Locate the finalized manifest.yaml, raising a specific DatasetError otherwise.

    Per DES-004, manifest.yaml is written on disarm via an atomic rename from
    manifest.yaml.part. A dataset with only the .part file present never
    finished recording and is invalid by definition.
    """
    dataset_dir = pathlib.Path(dataset_dir)
    manifest_path = dataset_dir / MANIFEST_FILENAME
    partial_path = dataset_dir / PARTIAL_MANIFEST_FILENAME
    if manifest_path.is_file():
        return manifest_path
    if partial_path.is_file():
        raise DatasetError(
            f"dataset is invalid: only a partial manifest ({PARTIAL_MANIFEST_FILENAME}) is "
            f"present in {dataset_dir} -- recording never finalized (disarm did not rename "
            f"it to {MANIFEST_FILENAME})"
        )
    raise DatasetError(f"dataset is invalid: no {MANIFEST_FILENAME} found in {dataset_dir}")


def load_manifest(dataset_dir) -> Dict[str, Any]:
    """Load and schema-validate manifest.yaml. Raises DatasetError on any problem."""
    manifest_path = find_manifest_path(dataset_dir)
    try:
        with open(manifest_path, "r") as f:
            manifest = yaml.safe_load(f)
    except yaml.YAMLError as exc:
        raise DatasetError(f"{manifest_path} is not parseable YAML: {exc}") from exc

    if not isinstance(manifest, dict):
        raise DatasetError(f"{manifest_path} did not parse to a mapping")

    validate_manifest_schema(manifest)
    return manifest


def validate_manifest_schema(manifest: Dict[str, Any]) -> None:
    """Validate required DES-004 manifest keys are present. Unknown keys are ignored (FE-1)."""
    missing = [k for k in REQUIRED_MANIFEST_KEYS if k not in manifest]
    if missing:
        raise DatasetError(f"manifest.yaml missing required keys: {missing}")

    try:
        format_version = int(manifest["format_version"])
    except (TypeError, ValueError) as exc:
        raise DatasetError("manifest.yaml format_version is not an integer") from exc
    # FE-1: accept this version and any newer, additive-compatible version;
    # only reject a version older than what this reader understands.
    if format_version < MIN_SUPPORTED_FORMAT_VERSION:
        raise DatasetError(
            f"manifest.yaml format_version {format_version} is older than the minimum "
            f"supported version {MIN_SUPPORTED_FORMAT_VERSION}"
        )

    intrinsics = manifest.get("camera_intrinsics")
    if not isinstance(intrinsics, dict):
        raise DatasetError("manifest.yaml camera_intrinsics must be a mapping")
    missing_intrinsics = [k for k in REQUIRED_CAMERA_INTRINSICS_KEYS if k not in intrinsics]
    if missing_intrinsics:
        raise DatasetError(f"manifest.yaml camera_intrinsics missing keys: {missing_intrinsics}")

    overlaps = manifest.get("overlaps")
    if not isinstance(overlaps, dict):
        raise DatasetError("manifest.yaml overlaps must be a mapping")
    missing_overlaps = [k for k in REQUIRED_OVERLAP_KEYS if k not in overlaps]
    if missing_overlaps:
        raise DatasetError(f"manifest.yaml overlaps missing keys: {missing_overlaps}")

    polygon = manifest.get("polygon")
    if not isinstance(polygon, list) or len(polygon) < 3:
        raise DatasetError("manifest.yaml polygon must be a list of at least 3 [x, y] vertices")
    for vertex in polygon:
        if not (isinstance(vertex, (list, tuple)) and len(vertex) == 2):
            raise DatasetError(f"manifest.yaml polygon vertex malformed: {vertex!r}")

    sha256_map = manifest.get("sha256")
    if not isinstance(sha256_map, dict) or not sha256_map:
        raise DatasetError("manifest.yaml sha256 must be a non-empty mapping of relpath -> hash")


def _opt_float(value) -> Optional[float]:
    """Parse an optional float: an empty/whitespace/None cell -> None; else float()."""
    if value is None or str(value).strip() == "":
        return None
    return float(value)


def _opt_int(value) -> Optional[int]:
    """Parse an optional int: an empty/whitespace/None cell -> None; else int()."""
    if value is None or str(value).strip() == "":
        return None
    return int(value)


def read_poses(dataset_dir) -> List[PoseRow]:
    """Read poses.csv by column NAME (FE-1). Raises DatasetError on missing file/columns/rows.

    Required kinematic columns (frame_idx, stamp_ns, x/y/z, qx/qy/qz/qw,
    pose_stamp_ns, sync_err_ms) hard-fail if unparseable. The GNSS columns
    (lat, lon, alt_amsl, gnss_stamp_ns) are Optional per DES-004 D4 / TS-05 --
    an empty cell parses to None instead of raising.
    """
    dataset_dir = pathlib.Path(dataset_dir)
    poses_path = dataset_dir / POSES_FILENAME
    if not poses_path.is_file():
        raise DatasetError(f"{POSES_FILENAME} not found in {dataset_dir}")

    rows: List[PoseRow] = []
    with open(poses_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise DatasetError(f"{POSES_FILENAME} has no header row")
        missing_cols = [c for c in REQUIRED_POSE_COLUMNS if c not in reader.fieldnames]
        if missing_cols:
            raise DatasetError(f"{POSES_FILENAME} missing required columns: {missing_cols}")

        for line_no, raw_row in enumerate(reader, start=2):
            extra = {k: v for k, v in raw_row.items() if k not in REQUIRED_POSE_COLUMNS}
            try:
                row = PoseRow(
                    frame_idx=int(raw_row["frame_idx"]),
                    stamp_ns=int(raw_row["stamp_ns"]),
                    x=float(raw_row["x"]),
                    y=float(raw_row["y"]),
                    z=float(raw_row["z"]),
                    qx=float(raw_row["qx"]),
                    qy=float(raw_row["qy"]),
                    qz=float(raw_row["qz"]),
                    qw=float(raw_row["qw"]),
                    pose_stamp_ns=int(raw_row["pose_stamp_ns"]),
                    lat=_opt_float(raw_row["lat"]),
                    lon=_opt_float(raw_row["lon"]),
                    alt_amsl=_opt_float(raw_row["alt_amsl"]),
                    gnss_stamp_ns=_opt_int(raw_row["gnss_stamp_ns"]),
                    sync_err_ms=float(raw_row["sync_err_ms"]),
                    extra=extra,
                )
            except (TypeError, ValueError) as exc:
                raise DatasetError(f"{POSES_FILENAME} row {line_no} failed to parse: {exc}") from exc
            rows.append(row)

    if not rows:
        raise DatasetError(f"{POSES_FILENAME} has no data rows")
    return rows


def validate_frame_idx_contiguous(poses: List[PoseRow], start: Optional[int] = None) -> None:
    """Verify frame_idx is contiguous starting at `start` (defaults to the first row's index)."""
    if not poses:
        raise DatasetError("cannot validate frame_idx contiguity: no pose rows")
    if start is None:
        start = poses[0].frame_idx
    indices = [p.frame_idx for p in poses]
    expected = list(range(start, start + len(indices)))
    if indices != expected:
        shown = indices[:10]
        suffix = "..." if len(indices) > 10 else ""
        raise DatasetError(
            f"poses.csv frame_idx is not contiguous starting at {start}: got {shown}{suffix}"
        )


def load_dataset(dataset_dir) -> Dataset:
    """Load and validate a full DES-004 dataset (manifest + poses). Raises DatasetError."""
    dataset_dir = pathlib.Path(dataset_dir)
    manifest = load_manifest(dataset_dir)
    poses = read_poses(dataset_dir)
    return Dataset(root=dataset_dir, manifest=manifest, poses=poses)


def sha256_of_file(path, chunk_size: int = 1 << 20) -> str:
    """Compute the sha256 hex digest of a file on disk."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            chunk = f.read(chunk_size)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()
