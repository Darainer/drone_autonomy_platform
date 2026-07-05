# Photogrammetry Pipeline — `tools/photogrammetry`

Post-flight photogrammetry pipeline for CAP-001 survey missions
([DES-005](../../docs/design/DES-005-photogrammetry-pipeline.md), dual-target).
Turns a DES-004 survey dataset (recorded on-drone by `survey_recorder_node`,
[DES-004](../../docs/design/DES-004-survey-dataset-recording.md)) into a
georeferenced point cloud, textured mesh, and orthophoto, gated by a
coverage-percentage check against the survey polygon.

> This is a standalone Python package, not a ROS 2 node — the interface is a
> file-system contract (a dataset directory in, a `products/` tree out). See
> `../../docs/design/DES-005-photogrammetry-pipeline.md` for the full design
> record and [TP-002](../../docs/test_plans/TP-002-survey-mapping.md) TS-08..TS-11
> for the verification specs referenced throughout this README.

---

## Two modes

| Mode | What it does | Requirement | Reconstruction? |
|---|---|---|---|
| `check` | Validate the dataset (manifest/checksums/poses via `verify_dataset.py`), compute sync-error stats, and predict coverage % by projecting each frame's camera footprint (from the manifest's camera intrinsics + recorded pose) and unioning against the survey polygon | MAP-7 | **No** — pure numpy/shapely geometry, no image pixels read beyond the sha256 check |
| `full` | Run `check` first (fail fast on an invalid dataset), then an OpenDroneMap (ODM) SfM/MVS reconstruction, then post-reconstruction coverage QA from the real orthophoto, then write the final report | MAP-4 (ground), MAP-8 (onboard) | **Yes** — the heavy step |

Both modes exit `0` iff the dataset is valid **and** the (predicted or
reconstructed) coverage meets the gate (default 95%, `--min-coverage` to
override), and both write `<dataset>/products/report/coverage.json` +
`coverage.md`.

## Two execution targets

Per DES-005, the same package/CLI runs unmodified on:

- **x86_64 ground station** — the reference hardware that meets the
  STK-1 ≤ 2 h product deadline for `--mode full`.
- **arm64 Jetson Orin Nano-class companion** — the on-drone target, `--mode
  check` as the post-landing consistency step (MAP-7), and `--mode full` as
  an on-device option (MAP-8, v1 manual invocation — see
  [DES-005 D5](../../docs/design/DES-005-photogrammetry-pipeline.md#design-decisions-fixed)).

`run_pipeline.py` detects which target it's running on
(`detect_execution_target()`, normalizing `x86_64`/`amd64` and
`aarch64`/`arm64`) purely for logging/report purposes — the ODM container
invocation itself is identical on both (the `opendronemap/odm` image is
itself published multi-arch).

## Usage

```bash
# Onboard post-landing consistency check (MAP-7) -- no reconstruction.
python run_pipeline.py --mode check --dataset /data/surveys/survey_<mission_id>

# Full reconstruction + coverage QA (MAP-4 ground station / MAP-8 onboard).
python run_pipeline.py --mode full --dataset /data/surveys/survey_<mission_id>

# Optional: override the coverage gate threshold (default 95.0).
python run_pipeline.py --mode check --dataset <dir> --min-coverage 90

# Standalone dataset validator (offload verification aid, no coverage math).
python verify_dataset.py /data/surveys/survey_<mission_id>
```

Exit code is the pass/fail verdict in both cases (0 = pass) — script this
directly rather than parsing stdout.

## Input contract: DES-004 dataset

```
survey_<mission_id>/
  manifest.yaml     # format_version, mission_id, polygon (local ENU, z ignored),
                     # altitude, overlaps{forward,side}, start_utc/end_utc,
                     # frame_count, camera_intrinsics{fx,fy,cx,cy,width,height},
                     # sha256 (relpath -> hex digest for every images/*.jpg + poses.csv)
  images/
    <frame_idx:06d>_<stamp_ns>.jpg
  poses.csv          # frame_idx, stamp_ns, x, y, z, qx, qy, qz, qw, pose_stamp_ns,
                      # lat, lon, alt_amsl, gnss_stamp_ns, sync_err_ms
```

`manifest.yaml` is written atomically (rename from `manifest.yaml.part`) on
disarm; a dataset with only the `.part` file is invalid by definition
(`verify_dataset.py` / `photogrammetry/dataset.py` enforce this). The reader
is forward-compatible (FE-1): unknown extra manifest keys or `poses.csv`
columns are ignored, never cause a validation failure.

## Output layout: `products/` (D6)

```
<dataset>/products/
  odm_georeferenced_model.laz   # point cloud
  odm_textured_model_geo.obj    # textured mesh
  odm_textured_model_geo.mtl    # mesh material
  orthophoto.tif                # georeferenced orthophoto (GeoTIFF)
  report/
    coverage.json                # machine-readable: coverage_pct, threshold, verdict, ...
    coverage.md                  # human-readable render of the same
```

`--mode check` only ever writes `report/` (no reconstruction products);
`--mode full` writes the full set above.

## Building the multi-arch image

```bash
docker buildx build --platform linux/amd64,linux/arm64 \
    -t <registry>/photogrammetry:<tag> tools/photogrammetry
```

`tools/photogrammetry/Dockerfile` is a **separate** image from the
repository-root ROS/colcon Dockerfile — this package is pure Python (plus a
`docker` CLI to launch the sibling ODM container), no ROS/C++ dependency, so
it does not extend or touch the root image. Base is `python:3.11-slim`
(official multi-arch manifest); no arch-hardcoded base tag.

### ODM version pin

Reconstruction itself runs in a **separate**, already-multi-arch
`opendronemap/odm` container, invoked at runtime by `odm_runner.py` as an
unattended `docker run` (sibling-container pattern — this image's own
`docker` CLI only launches that container, it does not bundle ODM). Current
pin (DES-005 D1):

```
opendronemap/odm:3.5.4
```

Keep `tools/photogrammetry/Dockerfile`'s comment and
`photogrammetry/odm_runner.py`'s `ODM_IMAGE` constant in lockstep. Any
*engine* version change (not just a patch bump) should go back through the
same review the initial engine choice got (DES-005 "Open questions").

## Runtime expectations

| Mode | Target | Budget | Requirement |
|---|---|---|---|
| `check` | Jetson Orin (arm64) | ≤ 15 min for a 1000-frame dataset | MAP-7, TS-10 (Jetson HIL) — in practice minutes of numpy/shapely work, no reconstruction |
| `full` | Ground station (x86_64) | ≤ 2 h dataset-to-product | STK-1 |
| `full` | Jetson Orin (arm64) | **Documented, not bounded** in v1 | MAP-8, TS-11 (wall time recorded in the run report, not gated) |

## Assumptions & v1 limitations (for designer review)

1. **Nadir / no-lever-arm camera footprint convention.** DES-004 records
   only the vehicle body pose (MAVROS `local_position/pose`) — no separate
   camera-to-body extrinsic, gimbal angle, or lever-arm offset is captured.
   `photogrammetry/footprint.py` therefore treats the camera as rigidly
   mounted with **no position offset** from the recorded pose and a fixed
   boresight convention (identity pose quaternion = straight-down nadir).
   If a future task adds a recorded camera extrinsic to DES-004, it composes
   with this fixed boresight; until then this is the documented v1
   convention for both `check` and `full` mode footprint math.
2. **Full-mode coverage QA uses the orthophoto's valid-data BOUNDING BOX**,
   not its true (possibly concave) footprint shape
   (`odm_runner.orthophoto_footprint()`: alpha-channel valid-data extent ->
   axis-aligned bounding box). This can **overestimate** coverage if the
   reconstruction has interior holes (e.g. a patch of failed feature
   matching in the middle of an otherwise-covered polygon) — the bounding
   box "covers" the hole even though no real reconstruction data exists
   there. Hole-accurate footprint extraction (e.g. alpha-shape or per-pixel
   validity against the survey polygon rather than a bbox) is a HIL-gate
   refinement, deferred out of v1 scope.

## Reference test fixture (TP-002 F2)

`tests/fixtures/survey_ref_rect_sample/` is the checked-in ~20-frame DES-004
dataset flown as a lawnmower pattern over `REF-RECT` (TP-002 F1: 100 m x
80 m rectangle) at the TP-002 common parameters (altitude 40 m,
forward_overlap 0.75, side_overlap 0.60, capture_rate 2 Hz, ~55 m x 42 m
nadir footprint). Regenerate it with:

```bash
python tests/fixtures/generate_sample_dataset.py
```

The same generator (`tests/fixtures/generate_sample_dataset.py`) parameterizes
rectangle size / altitude / overlaps and is used to produce the larger
1000-frame synthetic dataset for TS-10 (Jetson HIL) on demand — see the
module docstring for the exact scaling call; that larger dataset is
generated on the HIL bench, not checked in.

Tests: `tests/test_reference_dataset.py` asserts the checked-in fixture
validates (`verify_dataset()`) and passes the `--mode check` coverage gate
(≥ 95%), run against a tmp copy so the checked-in fixture is never mutated
with generated `products/`.
