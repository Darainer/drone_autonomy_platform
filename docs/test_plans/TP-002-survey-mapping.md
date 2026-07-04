# TP-002 — Survey Mapping Verification & Validation Test Plan

**Status:** Draft
**Requirements document:** [docs/requirements/platform_requirements.sdoc](../requirements/platform_requirements.sdoc) (MAP-*), [docs/requirements/stakeholder_requirements.sdoc](../requirements/stakeholder_requirements.sdoc) (STK-1)
**Designs under test:** [DES-003](../design/DES-003-survey-mission-coverage-trajectory.md), [DES-004](../design/DES-004-survey-dataset-recording.md), [DES-005](../design/DES-005-photogrammetry-pipeline.md)

## Scope

Verification of MAP-1..MAP-8 and validation of STK-1 for capability CAP-001.
Companion to the [CAP-001 implementation plan](../capabilities/CAP-001-implementation-plan.md).
Each coverage row references a full test specification (TS-*) below — per the
designer's review direction, a one-line row is not reviewable; the TS is the
contract the implementing agent codes against. Linkage is checked by
`scripts/check_traceability.py`.

## Requirements Coverage

| UID | Method | Environment | Test spec | WP | Status |
|---|---|---|---|---|---|
| MAP-6 | Test | SITL | TS-01, TS-02 | WP-1 | planned |
| MAP-1 ⚠ | Test | Unit + SITL | TS-03, TS-04 | WP-1 | planned |
| MAP-2 | Test | SITL (replay) | TS-05, TS-06 | WP-2 | planned |
| MAP-3 | Test | Unit | TS-07 | WP-2 | planned |
| MAP-4 | Test | Ground station | TS-08 | WP-3 | planned |
| MAP-5 | Test | Unit + SITL e2e | TS-09, TS-12 | WP-3/4 | planned |
| MAP-7 | Test | Jetson HIL | TS-10 | WP-3 | planned |
| MAP-8 | Test | Jetson HIL | TS-11 | WP-3 | planned |
| STK-1 | Demonstration | Field | TS-13 | WP-4 | planned |

⚠ MAP-1 is `safety-critical`: Test method mandatory, rows may not remain
`planned` when WP-1 ships (`check_traceability.py --strict` enforces once
MAP-1 is Approved).

## Reference fixtures

- **F1 — reference polygons:** `REF-RECT` 100 m × 80 m rectangle; `REF-HEX`
  convex 6-vertex polygon (~0.8 ha); `REF-BAD-*` invalid set (2 vertices,
  self-intersecting bowtie). Checked in with WP-1 as test data.
- **F2 — reference sample dataset:** ~20-frame dataset in DES-004 format over
  `REF-RECT` (synthetic renders or bench captures), checked in with WP-3 at
  `tools/photogrammetry/tests/fixtures/`.
- **F3 — replay bag:** rosbag with `/oak/rgb/image_raw` +
  `/mavros/local_position/pose` + `/mavros/global_position/global` at
  realistic rates/timestamps, checked in with WP-2.
- Common parameters unless a TS says otherwise: altitude 40 m,
  forward_overlap 0.75, side_overlap 0.60, survey_speed 5 m/s,
  capture_rate 2 Hz, OAK-D FOV 69°/55° (footprint ≈ 55 m × 42 m at 40 m AGL).

## Test Specifications

### TS-01 — Survey mission dispatch (`Verifies: MAP-6`)
- **Item under test:** `autonomy_node` survey handling (DES-003 T1.3)
- **Environment:** SITL / ROS graph with autonomy_node only
- **Setup:** publish a valid survey `Mission` (F1 `REF-RECT`, common params) on `~/survey_request`
- **Procedure:** capture `/mission` and `/mission_status` for 5 s
- **Pass:** exactly one `/mission` with `mission_type == "survey"`, all spec fields byte-identical to the request, non-empty unique `mission_id`; exactly one `MissionStatus{state:"active"}` with the same `mission_id`; no further dispatches.

### TS-02 — Degenerate survey rejection (`Verifies: MAP-6`)
- **Item under test:** DES-003 validation rules
- **Setup:** publish each `REF-BAD-*` polygon, plus altitude 5 m and 200 m cases, plus overlap 0.1 and 0.99 cases, plus capture_rate 0.1 and 20 Hz cases, plus a speed/rate inconsistency case (12 m/s at 0.5 Hz — violates `speed ≤ s_cap × rate`)
- **Pass:** for every case: zero `/mission` publications; one `MissionStatus{state:"rejected"}` whose `detail` names the failed rule.

### TS-03 — Coverage geometry properties (`Verifies: MAP-1`)
- **Item under test:** `SurveyPlanner` (pure geometry, DES-003 T1.4)
- **Environment:** unit (gtest, no ROS graph)
- **Procedure:** generate plans for `REF-RECT` and `REF-HEX` at the common parameters and at the parameter extremes (alt 10/120 m, overlaps 0.30/0.95)
- **Pass (all cases):**
  - adjacent lane spacing ≤ `(1 − side_overlap) × W` + 1 cm tolerance;
  - along-track waypoint spacing ≤ `(1 − forward_overlap) × H` + 1 cm;
  - every waypoint within polygon dilated by `W/2`;
  - union of per-waypoint footprints covers ≥ 99% of the polygon area (shapely/CGAL check in the test);
  - all waypoint z == `survey_altitude_m`; yaw parallel to lane direction ± 1°.

### TS-04 — Survey chain in SITL (`Verifies: MAP-1`)
- **Item under test:** autonomy → navigation → `/trajectory` chain (DES-001 wiring + DES-003)
- **Environment:** SITL via the `run_simulation` stage
- **Procedure:** dispatch `REF-RECT` survey; record `/trajectory`
- **Pass:** one `Trajectory` whose path passes the TS-03 assertions; latency dispatch→trajectory < 5 s.

### TS-05 — Frame/pose sync budget (`Verifies: MAP-2`)
- **Item under test:** `survey_recorder_node` sync (DES-004 D4)
- **Environment:** replay F3 with an armed survey mission
- **Procedure:** replay 60 s; disarm via `MissionStatus{complete}`; read dataset
- **Pass:** dataset contains ≥ 95% of the expected `rate × 60` frames at the **mission-specified** capture rate (repeat at 1 Hz and 4 Hz to prove the field is honored); every `poses.csv` row has `sync_err_ms ≤ 50`; `lat/lon` non-null for ≥ 99% of rows; `dropped_sync` in manifest equals replayed pairs exceeding slop.

### TS-06 — Recording window (`Verifies: MAP-2`)
- **Item under test:** DES-004 D6 trigger semantics
- **Procedure:** replay F3 for 30 s *before* arming, 30 s armed, 30 s after `complete`
- **Pass:** all recorded frame timestamps fall inside the armed window ± 1 frame period; a second arm creates a new dataset directory, never appends.

### TS-07 — Dataset format & offload verifiability (`Verifies: MAP-3`)
- **Item under test:** DES-004 format writer + `verify_dataset.py`
- **Environment:** unit (pytest)
- **Procedure:** (a) validate a TS-05-produced dataset; (b) corrupt one image byte; (c) delete the manifest
- **Pass:** (a) `verify_dataset.py` exit 0 — schema fields present, checksums match, frame_idx contiguous; (b) exit ≠ 0 naming the corrupt file; (c) dataset reported invalid (finalized manifest required per DES-004).

### TS-08 — Full-mode pipeline smoke (`Verifies: MAP-4`)
- **Item under test:** `run_pipeline.py --mode full` (DES-005)
- **Environment:** ground-station class x86_64 CI runner (container)
- **Procedure:** run on F2 with zero interactive input (stdin closed)
- **Pass:** exit 0; `products/` contains non-empty point cloud (`.laz`), textured mesh (`.obj` + material), orthophoto (`.tif`), `report/coverage.json` with numeric `coverage_pct`; no prompt ever written to stdout (assert on captured output).

### TS-09 — Coverage-QA known-hole (`Verifies: MAP-5`)
- **Item under test:** `photogrammetry/coverage.py`
- **Environment:** unit (pytest)
- **Procedure:** synthetic reconstruction footprint = `REF-RECT` minus a centered 10%-area hole
- **Pass:** reported coverage in [88%, 92%]; gate verdict `fail` at threshold 95%; full-polygon input reports ≥ 99.9% and verdict `pass`.

### TS-10 — Onboard consistency check budget (`Verifies: MAP-7`)
- **Item under test:** `run_pipeline.py --mode check` on the companion
- **Environment:** Jetson Orin HIL
- **Procedure:** run against a 1000-frame synthetic dataset (F2 generator scaled up)
- **Pass:** wall time ≤ 15 min (budget from MAP-7); report contains predicted coverage %, sync-error stats, completeness verdict; exit code reflects the ≥ 95% predicted-coverage gate.

### TS-11 — Onboard full reconstruction (`Verifies: MAP-8`)
- **Item under test:** arm64 pipeline container (DES-005 D5)
- **Environment:** Jetson Orin HIL
- **Procedure:** `--mode full` on F2, unattended
- **Pass:** exit 0 with the same product set as TS-08 (byte-identical structure, not bytes); wall time recorded in the run report (documented, not gated — MAP-8 has no time bound).

### TS-12 — End-to-end SITL survey (`Verifies: MAP-5`)
- **Item under test:** full chain WP-1+2+3 (WP-4 integration)
- **Environment:** SITL via `run_simulation` stage + pipeline container
- **Procedure:** fly `REF-RECT` survey in SITL with simulated camera; offload dataset; `--mode full`
- **Pass:** `coverage_pct ≥ 95` in `report/coverage.json`, asserted automatically; zero manual steps between dataset and report (single script).

### TS-13 — Field demonstration (STK-1 validation)
- **Method:** Demonstration (manual, dated report via the `report` skill)
- **Procedure:** one survey flight over a real polygon; onboard `--mode check` before leaving site; ground-station `--mode full`
- **Pass = STK-1 acceptance:** (a) coverage ≥ 95%; (b) GSD ≤ 5 cm; (c) product ≤ 2 h after landing on reference GS hardware; (d) no manual intervention between offload and product; **plus** onboard check verdict available ≤ 15 min after landing (MAP-7 in the field).

## Test Environments

| Environment | How |
|---|---|
| Unit (gtest / pytest) | Package tests in `src/navigation`, `src/mapping`; pipeline tests in `tools/photogrammetry/tests/`; run in CI |
| Smoke | `scripts/smoke_test.sh` — extend node list with `survey_recorder_node` (WP-2) |
| SITL | `sim-test` agent via the workflow's built-in `run_simulation` stage — scenario + pass criteria in the submitted plan text, **not** a plan step (see implementation-plan harness constraints) |
| Ground station | x86_64 CI runner / reference GS hardware (DES-005) |
| Jetson HIL | Manual-triggered runs on the Orin; results recorded in `docs/reports/` |
| Field | Demonstration flight; dated report in `docs/reports/` |

## Exit Criteria

- All Test-method rows implemented with `Verifies:` markers and passing; each
  implemented test references its TS id in a comment.
- MAP-1 rows green under `python scripts/check_traceability.py --strict`.
- Gap report `docs/reports/gap_CAP-001.md` reads all target elements present.
- TS-13 report filed in `docs/reports/` with all four STK-1 criteria met.
