# DES-005 — Post-Flight Photogrammetry Pipeline (Dual-Target)

**Status:** Draft (approved together with the CAP-001 implementation plan at the WP-level human gate)
**Safety-critical:** no
**Capability:** CAP-001 · **Work packages:** WP-3 (build), WP-4 (validation)
**Test specs:** TP-002 TS-08..TS-12

## Summary

One pipeline package, `tools/photogrammetry`, with two execution targets and
two modes. **Targets:** the reference ground station (x86_64, meets the STK-1
≤ 2 h product deadline) and the Orin Nano class companion (arm64) — per the
designer decision that the pipeline shall also be executable on device as a
post-flight step. **Modes:** `check` — fast dataset consistency/coverage
check without reconstruction (the onboard post-landing step, MAP-7) — and
`full` — SfM/MVS reconstruction + coverage QA (MAP-4 on the ground station,
MAP-8 onboard).

## Requirements addressed

| UID | How this design satisfies it |
|---|---|
| MAP-4 | `run_pipeline.py --mode full`: dataset → georeferenced point cloud + textured mesh + coverage report, unattended |
| MAP-5 | Coverage-QA module computes % of survey polygon covered; machine-readable report gates at 95% |
| MAP-7 | `run_pipeline.py --mode check` on the companion: consistency report ≤ 15 min after mission complete |
| MAP-8 | Same package/CLI runs `--mode full` on arm64/Jetson (container build for both arches) |

## Current state

`tools/` does not exist. Input format is fixed by DES-004. The Orin runs
JetPack 6.x with Docker (see `docker/Dockerfile.dev`).

## Design decisions (fixed)

| # | Decision | Choice | Rejected alternative |
|---|---|---|---|
| D1 | SfM/MVS engine | **OpenDroneMap (ODM) via container** (`opendronemap/odm`, amd64 + arm64 published) | COLMAP — better research control but no unattended end-to-end (mesh/ortho) path without significant glue, and CUDA build on Jetson is a maintenance burden |
| D2 | Wrapper | **Python CLI `run_pipeline.py`** (single command, exit code = verdict), pure-Python deps for `check` mode (numpy, shapely, PyYAML, Pillow) | Bash orchestration — no unit-testable QA logic |
| D3 | Consistency check (MAP-7) | **No reconstruction**: manifest/checksum validation, sync-error stats, then projected-footprint union (poses × camera model from manifest intrinsics) vs. survey polygon → predicted coverage % | Fast partial reconstruction — cannot bound to 15 min on arm64 across dataset sizes |
| D4 | Coverage QA after reconstruction (MAP-5) | **Orthophoto/point-cloud footprint polygon vs. survey polygon** (shapely intersection-over-polygon) | Mesh-based occlusion analysis — overkill for a coverage percentage |
| D5 | Onboard packaging | **arm64 build of the same pipeline container, preinstalled in the companion image** (`docker/` addition); triggered manually or by a post-mission service hook — v1 is manual invocation | Automatic in-flight processing — power/thermal contention with perception during flight; post-flight only |
| D6 | Outputs | `odm_georeferenced_model.laz` (point cloud), `odm_textured_model_geo.obj` (mesh), `orthophoto.tif`, `report/coverage.json` + `report/coverage.md` | — |

## Proposed design

```
tools/photogrammetry/
  run_pipeline.py        # CLI: --mode {check,full} --dataset DIR [--polygon-from-manifest]
  verify_dataset.py      # standalone manifest/checksum validation (offload aid, MAP-3)
  photogrammetry/
    dataset.py           # DES-004 format reader + schema validation
    footprint.py         # camera model, per-frame ground footprint, union coverage
    coverage.py          # polygon coverage % + report writer (used by both modes)
    odm_runner.py        # ODM container invocation, product collection
  tests/                 # pytest, runs in CI (TS-08, TS-09)
  Dockerfile             # multi-arch (amd64/arm64) wrapper image pinning ODM version
  README.md              # usage, both targets, runtime expectations
```

`--mode check` (MAP-7): validate → footprint coverage prediction → report;
exit 0 iff dataset valid AND predicted coverage ≥ threshold (default 95%).
Runtime target on Orin: ≤ 15 min for a 1000-frame dataset (TS-10; it is
minutes of numpy/shapely work in practice).

`--mode full` (MAP-4/MAP-8): `check` stage first (fail fast), then ODM run
(georeferencing from `poses.csv` GNSS columns via ODM geo files), then
post-reconstruction coverage QA (D4), then report. No prompts, no manual
steps (STK-1(d)); exit 0 iff products exist AND coverage ≥ 95%.

Traceability markers land in code: `# Implements: MAP-4` (`odm_runner.py`
orchestration), `# Implements: MAP-5` (`coverage.py`), `# Implements: MAP-7`
(`run_pipeline.py` check mode), `# Implements: MAP-8` (Dockerfile/arm64 entry
documented in code + README; marker on the arch-dispatch code in
`run_pipeline.py`).

## Interfaces

Not a ROS component — file-system contract only: input = DES-004 dataset
directory; output = `<dataset>/products/` tree (D6). The gap checker tracks
it as `kind: offboard, path: tools/photogrammetry`; onboard execution is a
behavior (`Implements: MAP-7/MAP-8`), not a separate container.

## Safety impact

None. Onboard execution is post-flight only (D5) — never concurrent with
armed flight; no interaction with the control path.

## Test strategy

TP-002: TS-08 (full-mode smoke on the reference sample dataset),
TS-09 (coverage-QA unit, known hole), TS-10 (onboard check-mode budget,
Jetson HIL), TS-11 (onboard full-mode execution, Jetson HIL), TS-12
(SITL end-to-end, WP-4). Reference sample dataset (~20 frames, DES-004
format) is checked in under `tools/photogrammetry/tests/fixtures/`.

## Alternatives considered

See decisions table. Also rejected: separate onboard implementation of the
consistency check inside `src/mapping` (C++) — duplicates the footprint/QA
math that `full` mode needs anyway; one Python package serves both targets
(MAP-8 explicitly demands executability, not a rewrite).

## Open questions

None — D1–D6 fixed for WP-3. ODM version pin chosen at T3.2 implementation
time is recorded in the Dockerfile and README (any *engine change* goes back
to the designer).
