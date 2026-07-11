# TP-003 — Pre-Mow Field Clearance Verification & Validation Test Plan

**Status:** Draft
**Requirements document:** [docs/requirements/platform_requirements.sdoc](../requirements/platform_requirements.sdoc) (CLR-*), [docs/requirements/stakeholder_requirements.sdoc](../requirements/stakeholder_requirements.sdoc) (STK-2)
**Designs under test:** [DES-007](../design/DES-007-clearance-sensing-and-model.md), [DES-008](../design/DES-008-clearance-mission-and-confirmation.md), [DES-009](../design/DES-009-tracker-node.md), [DES-010](../design/DES-010-clearance-recorder.md), [DES-011](../design/DES-011-clearance-report-and-delivery.md)

## Scope

Verification of CLR-1..CLR-12 and validation of STK-2 for capability
CAP-002. Companion to the
[CAP-002 implementation plan](../capabilities/CAP-002-implementation-plan.md).
Each coverage row references a full test specification (TS-*) below; the TS
is the contract the implementing agent codes against. Linkage is checked by
`scripts/check_traceability.py`.

## Requirements Coverage

| UID | Method | Environment | Test spec | WP | Status |
|---|---|---|---|---|---|
| CLR-1 | Test | SITL | TS-01, TS-02 | WP-B | planned |
| CLR-8 ⚠ | Test | Unit + SITL | TS-03, TS-04 | WP-B | planned |
| CLR-10 ⚠ | Test | SITL | TS-11 | WP-B | planned |
| CLR-3 | Test | Unit + replay | TS-05 | WP-C | planned |
| CLR-4 | Test | Unit (synthetic) | TS-06 | WP-D | planned |
| CLR-5 | Test | Replay + unit | TS-07, TS-08 | WP-D | planned |
| CLR-6 | Test | Companion-class CI | TS-09 | WP-E | planned |
| CLR-7 | Test | Container e2e | TS-10 | WP-E | planned |
| CLR-2 | Test | Eval harness (GPU) | TS-12 | WP-A | planned |
| CLR-9 | Inspection + Test | CI | TS-13 | WP-A | planned |
| CLR-11 | Test | Bench (hardware) | TS-14 | WP-G | planned |
| CLR-12 | Test | Eval harness + bench | TS-15 | WP-H | planned |
| STK-2 (a–c, e) | Test | SITL e2e | TS-16, TS-17 | WP-F | planned |
| STK-2 (a–e) | Demonstration | Field (day) | TS-18 | WP-F | planned |
| STK-2 (f) | Demonstration | Field (dawn) | TS-19 | WP-F | planned |

⚠ CLR-8 and CLR-10 are `safety-critical`: Test method mandatory; rows may
not remain `planned` when WP-B ships (`check_traceability.py --strict`
enforces once they are Approved).

## Reference fixtures

- **F1 — reference polygons:** reuse TP-002 F1 (`REF-RECT`, `REF-HEX`,
  `REF-BAD-*`); add **`REF-AREAS`** — three named areas
  (`north_field` 120×80 m, `paddock` REF-HEX, `strip` 200×30 m) with
  ~150 m transit separations.
- **F4 — reference findings store:** DES-010 `format_version: 1` store with
  20 findings (mixed classes/areas, 2 outside-area, 3 unconfirmed), full
  `track.csv`, checked in at `tools/clearance_report/tests/fixtures/`
  (WP-E; a truncated-manifest variant included).
- **F5 — clearance replay bag:** rosbag with `/oak/rgb/image_raw`,
  `/oak/rgb/camera_info`, `/detections` (scripted synthetic detections for
  3 objects incl. one 15-frame occlusion gap and one 2-frame flicker),
  `/mavros/local_position/pose`, `/mavros/global_position/global` at
  realistic rates (WP-C/WP-D).
- **F6 — geolocation scenes:** synthetic (pose, camera_info, bbox) tuples
  with analytically known ground positions: nadir + 30° oblique, 15 m and
  22 m AGL, attitude perturbations up to 3°, positions inside/outside/on
  `REF-AREAS` boundaries (WP-D).
- **F7 — SITL clearance world:** Gazebo world with `REF-AREAS`, static
  person + animal + vehicle models placed inside areas (2 per area), one
  low-confidence plant (small/partially occluded model), one model outside
  all areas (WP-F).
- Common parameters unless a TS says otherwise: day profile 15 m /
  6 m/s / overlaps 0.30; dawn profile 22 m; confirm threshold 0.80,
  budget 10; OAK-D FOV 69°/55°; thermal FOV 50°/40°.

## Test Specifications

### TS-01 — Clearance mission dispatch (`Verifies: CLR-1`)
- **Item:** `autonomy_node` clearance handling (DES-008 B2)
- **Environment:** ROS graph with autonomy_node only
- **Procedure:** publish valid clearance `Mission`s on `~/clearance_request`
  for both profiles over `REF-AREAS`; capture `/mission` + `/mission_status`
  5 s each
- **Pass:** per request: exactly one `/mission` with
  `mission_type=="clearance"`, all fields intact, profile-default altitude
  filled when 0 (15.0 day / 22.0 dawn), unique `clearance_<utc>` id;
  exactly one `MissionStatus{active}`; no further dispatches.

### TS-02 — Clearance rejection matrix (`Verifies: CLR-1`)
- **Item:** DES-008 validation rules (B2)
- **Procedure:** publish each invalid case: bad profile string; 0 and 9
  areas; one `REF-BAD-*` area among valid ones; duplicate / missing /
  empty area names; altitude 5 m and 80 m; `confirm_budget` 21
- **Pass:** per case: zero `/mission`; one `MissionStatus{rejected}` whose
  `detail` names the failed rule.

### TS-03 — Multi-area coverage geometry per profile (`Verifies: CLR-8`)
- **Item:** navigation clearance planning (DES-008 B3), `SurveyPlanner` reuse
- **Environment:** unit (gtest, no ROS graph)
- **Procedure:** plan `REF-AREAS` for both profiles; also single-area
  extremes (altitude 10/50 m)
- **Pass (all cases):** per area, TS-03/TP-002-style assertions with the
  profile camera model — lane spacing ≤ `(1−0.30)·W` + 1 cm; footprint
  union covers ≥ 95% of each area polygon; every waypoint z ==
  profile altitude; areas visited in request order; exactly one transit
  leg between consecutive areas at coverage altitude; effective GSD at
  each waypoint ≤ 4 cm/px (day, 512 input) / ≤ 4.5 cm/px native (dawn).

### TS-04 — Clearance chain in SITL (`Verifies: CLR-8`)
- **Item:** autonomy → navigation → `/trajectory` chain
- **Environment:** SITL via the `run_simulation` stage
- **Procedure:** dispatch `REF-AREAS` day-profile mission; record `/trajectory`
- **Pass:** one `Trajectory` passing TS-03 assertions; dispatch→trajectory
  latency < 5 s; dawn-profile repeat passes with thermal parameters.

### TS-05 — Persistent tracking (`Verifies: CLR-3`)
- **Item:** `byte_tracker.py` + `tracker_node` (DES-009)
- **Environment:** unit (pytest, synthetic sequences) + F5 replay
- **Procedure:** unit: scripted detection sequences; replay: F5 through the node
- **Pass:** one object → exactly one id over 100 frames; two crossing
  objects retain distinct ids; 15-frame gap (< `track_buffer`) re-associates
  to the same id; the 2-frame flicker never appears on `/tracked_objects`
  (min_hits=3); majority-class smoothing survives a 3-frame class flicker;
  per-frame update < 50 ms (COMP-4) at 30 synthetic tracks; empty input
  frames still publish empty arrays.

### TS-06 — Geolocation accuracy & area tagging (`Verifies: CLR-4`)
- **Item:** recorder ray-cast + containment (DES-010 D2)
- **Environment:** unit (gtest) on F6
- **Pass:** every F6 scene: computed WGS84 position within 5 m of analytic
  truth (within 1 m for unperturbed nadir cases — model correctness vs.
  budget); inside/outside `area_name` tags correct, boundary cases
  deterministic; origin-fix pairing uses the arm-time fix.

### TS-07 — Evidence clips & recording window (`Verifies: CLR-5`)
- **Item:** recorder clip pipeline (DES-010 D3–D5, D7)
- **Environment:** F5 replay with an armed clearance mission
- **Pass:** per tracked in-area object: exactly one finding, one clip dir;
  clip spans ≥ 3 s total with ≥ 1.0 s (± 1 frame at 10 fps) before the
  track's first confirmed frame; `clip.json` and the `findings.jsonl`
  record carry class, max confidence, track id, lat/lon, UTC stamps;
  outside-area object logged with `area_name:""` and absent from
  `/findings`; frames only within the armed window; second arm → new store
  directory; `/findings` publishes on creation and on confidence upgrade.

### TS-08 — Findings-store schema & guards (`Verifies: CLR-5`)
- **Item:** store writer (DES-010 format)
- **Environment:** unit
- **Pass:** TS-07-produced store validates against the `format_version: 1`
  schema (manifest fields, checksums, `track.csv` at 1 Hz ± 10%,
  findings/clips cross-references resolve); deleting the manifest
  invalidates the store; `min_free_mb` below threshold → arm refused with
  ERROR diagnostic; write failure mid-mission → manifest finalized with
  `status: truncated`.

### TS-09 — Report build & verdicts (`Verifies: CLR-6`)
- **Item:** `build_report.py` (DES-011 D1–D3)
- **Environment:** arm64 companion-class CI container (ffmpeg present)
- **Procedure:** build on F4; on F4 with `track.csv` filtered to create a
  20% coverage hole in `north_field`; on the truncated-manifest variant
- **Pass:** exit 0; `report.zip` contains `index.html`, `verdicts.json`,
  `findings.geojson`, one mp4 per finding; verdicts: area with 0 findings
  + full coverage → `CLEAR`; `paddock` with findings → `FINDINGS(n)` with
  unconfirmed count; holed `north_field` → `COVERAGE_INCOMPLETE` with hole
  polygons in the GeoJSON (never `CLEAR`); modality note present;
  truncated store → report marked `DATA INCOMPLETE`; no stdin reads; wall
  time ≤ 4 min.

### TS-10 — Delivery end-to-end (`Verifies: CLR-7`)
- **Item:** `deliver.py` + `report_on_complete.py` + `communication_node`
  downlink (DES-011 D4–D7)
- **Environment:** docker-compose: stack + local ntfy container + ssh endpoint
- **Procedure:** publish `MissionStatus{complete}` for a mission whose F4
  store is in place; subscribe the ntfy topic; run a `Finding` through
  `/findings` with `gcs_host` set
- **Pass:** report.zip present at the endpoint and ntfy notification
  (per-area verdicts + link) received ≤ 5 min after `complete`; delivery
  failure exits ≠ 0 and is retried once; `communication_node` emits the
  JSON datagram matching the `Finding`.

### TS-11 — Confirmation tour (`Verifies: CLR-10`)
- **Item:** autonomy confirmation queue + navigation pass-through
  (DES-008 B4, B3.2)
- **Environment:** SITL via the `run_simulation` stage
- **Procedure:** day mission over `REF-AREAS`; inject `/findings` during
  coverage: 3 below threshold (0.5–0.7), 1 above (0.9); drive
  coverage-complete; also repeat with 12 low-confidence findings and
  budget 10, and one finding with a sub-8 m altitude request variant
- **Pass:** after coverage-complete, exactly one `clearance_confirm`
  mission with 3 waypoints at 10.0 m in nearest-neighbor order and the
  same `mission_id`; the 0.9 finding is not visited; `complete` only after
  tour completion; budget run visits exactly 10 and the excess 2 remain
  `unconfirmed` in the store; navigation rejects the sub-8 m variant
  (altitude floor); no confirmation dispatch when the queue is empty.

### TS-12 — E-RGB model gate (`Verifies: CLR-2`)
- **Item:** fine-tuned day-check model + eval harness (DES-007 §5.4, WP-A)
- **Environment:** GPU eval harness (training workstation CI)
- **Procedure:** run the harness on the held-out E-RGB set at the selected
  operating point (A3 decision); compare against the deployed-engine baseline
- **Pass:** per range bin and occlusion grade O0–O2: person recall ≥ 0.95,
  animal-class recall ≥ 0.85; false findings ≤ 1/ha on E-NEG; TensorRT
  latency ≤ 25 ms on Orin; no eval-set image from a training site;
  binned results table published in the model card.

### TS-13 — Dataset provenance gate (`Verifies: CLR-9`)
- **Item:** provenance manifest + tooling (WP-A A1)
- **Environment:** CI (pytest)
- **Pass:** every image source referenced by the deployed model's training
  config resolves to a manifest entry with license ∈ the commercial-allowed
  set; academic-flagged sets present → build of the *deployment* dataset
  fails; manifest schema validates; labeling-plan doc referenced and present.

### TS-14 — Thermal integration bench (`Verifies: CLR-11`)
- **Item:** thermal driver + sync (WP-G G2)
- **Environment:** bench (Jetson + sensor)
- **Pass:** `/thermal/image_raw` ≥ 8 Hz sustained 10 min, stamped on the
  Jetson clock; ApproximateTime pairing with pose ≤ 50 ms slop for ≥ 99%
  of frames; camera info published; recorder ingests thermal clips
  (16-bit PNG + preview per DES-010 D9).

### TS-15 — E-THERM gate incl. concealment (`Verifies: CLR-12`)
- **Item:** dawn-sweep detector (classical baseline or model, DES-007 D4)
- **Environment:** eval harness on E-THERM (heated-decoy campaigns T1)
- **Pass:** animal recall ≥ 0.90 per range bin **including O3 (fully
  concealed) placements**; false findings ≤ 1/ha on dawn E-NEG footage; if
  the classical baseline passes, the ML fallback (H4) is not built —
  gate recorded either way.

### TS-16 — SITL end-to-end, day profile (validates STK-2 a–c, e)
- **Item:** full chain WP-A..E merged
- **Environment:** SITL via `run_simulation` on F7
- **Pass:** unattended from one `clearance_request` to `report.zip`:
  every in-area planted model → exactly one finding (no duplicates, no
  misses), geolocated ≤ 5 m of its Gazebo pose; the low-confidence plant
  triggers a confirmation visit; the outside-area model yields no finding;
  clips ≥ 3 s with pre-roll; verdicts correct per TS-09 rules; zero
  interactive input end-to-end.

### TS-17 — SITL end-to-end, dawn profile (validates STK-2 + CLR-12 chain)
- **Item:** dawn chain with simulated thermal source
- **Environment:** SITL; `/thermal/image_raw` replayed/simulated
- **Pass:** TS-16 assertions with the thermal detector chain and dawn
  coverage parameters; thermal clips rendered in the report.

### TS-18 — Field trial, day final check (validates STK-2 a–e)
- **Item:** the capability on hardware (requires DES-006 merged)
- **Environment:** field; surrogate targets (mannequin incl. child-scale,
  animal decoys) at RTK-surveyed positions in graded grass O0–O2
- **Pass:** STK-2 (a)–(e) measured: coverage ≥ 95% with holes flagged; one
  finding per surrogate; geolocation ≤ 5 m vs survey; clips reviewable;
  report on the registered phone ≤ 5 min after landing; zero operator
  interaction post-dispatch. Results filed as a dated report (`report` skill).

### TS-19 — Field trial, dawn thermal sweep (validates STK-2 f)
- **Item:** dawn capability on hardware (requires WP-G)
- **Environment:** field at dawn thermal-contrast conditions; heated decoys
  (body-temperature pads) incl. ≥ 2 fully concealed O3 placements
- **Pass:** every heated decoy — including all O3 placements — produces a
  finding; report delivered per CLR-7. Results filed as a dated report.
