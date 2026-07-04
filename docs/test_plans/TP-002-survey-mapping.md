# TP-002 — Survey Mapping Verification & Validation Test Plan

**Status:** Draft
**Requirements document:** [docs/requirements/platform_requirements.sdoc](../requirements/platform_requirements.sdoc) (MAP-*), [docs/requirements/stakeholder_requirements.sdoc](../requirements/stakeholder_requirements.sdoc) (STK-1)

## Scope

Verification planning for the survey-mapping capability CAP-001 (MAP-1..MAP-6)
and validation of the stakeholder requirement STK-1. Companion to the
[CAP-001 implementation plan](../capabilities/CAP-001-implementation-plan.md):
each work package implements the rows listed against it and flips them from
`planned` to `implemented`. Linkage is checked by
`scripts/check_traceability.py`.

Numeric budgets marked *(DES)* are placeholders until the referenced design
doc fixes them; update the row when the DES doc is approved.

## Requirements Coverage

| UID | Method | Environment | Test / Procedure | WP | Status |
|---|---|---|---|---|---|
| MAP-6 | Test | SITL (`sim-test` queue) | Publish a survey mission spec (reference polygon, altitude 40 m, 75/60% overlap) to `autonomy_node`; assert exactly one `/mission` with `mission_type == "survey"` and the spec fields intact. Publish a degenerate polygon (< 3 vertices / self-intersecting); assert rejection with an error status and no `/mission` dispatch | WP-1 | planned |
| MAP-1 | Test | Unit + SITL | Geometry property tests on the generated coverage trajectory for the reference polygon: (a) adjacent lane spacing ≤ (1 − side_overlap) × camera footprint width at mission altitude; (b) along-track capture spacing ≤ (1 − forward_overlap) × footprint height; (c) every trajectory point inside the polygon dilated by ≤ 1 footprint; (d) union of per-point footprints covers ≥ 99% of the polygon. SITL: survey mission produces a `/trajectory` consumed by the DES-001 chain | WP-1 | planned |
| MAP-2 | Test | SITL (`sim-test` queue) | Replay camera frames on `/oak/rgb/image_raw` and pose on `/mavros/local_position/pose` during an armed survey mission; assert the dataset contains ≥ 95% of published frames, each with a pose whose timestamp delta ≤ sync budget *(DES-004)*; assert recording arms on survey start and disarms on survey end (no frames recorded outside the mission window) | WP-2 | planned |
| MAP-3 | Test | Unit (`pytest`) | Validate a recorded dataset against the documented manifest schema *(DES-004)*; assert the offload procedure produces a single self-contained package readable without the vehicle present | WP-2 | planned |
| MAP-4 | Test | Ground station | Run `tools/photogrammetry` on the reference sample dataset with a single command; assert exit 0 with **zero interactive prompts**, and that a georeferenced point cloud and textured mesh exist and are non-empty | WP-3 | planned |
| MAP-5 | Test | Unit + ground station | Coverage-QA unit test: synthetic reconstruction with a known 10% hole over the reference polygon reports 90% ± 2%. Pipeline-level: coverage report is machine-readable and gates pass/fail at 95% | WP-3 | planned |
| MAP-5 | Test | SITL end-to-end | Full chain: SITL survey over reference polygon → dataset → pipeline → coverage ≥ 95% asserted automatically | WP-4 | planned |
| STK-1 | Demonstration | Field test | One survey flight over a real polygon: verify acceptance (a) coverage ≥ 95%, (b) GSD ≤ 5 cm, (c) map product ≤ 2 h after landing on reference ground-station hardware, (d) no manual steps between offload and product. Results filed as a dated report in `docs/reports/` (`report` skill) | WP-4 | planned |

Methods: **Test** (automated, carries a `Verifies:` marker once implemented),
**Analysis**, **Inspection**, **Demonstration** (per DO-178C verification methods).

**Safety-critical note:** MAP-1 is tagged `safety-critical` (trajectory
generation commands the vehicle). Its rows must use the **Test** method, must
carry `Verifies: MAP-1` markers, and may not remain `planned` when WP-1
ships — `python scripts/check_traceability.py --strict` enforces this once
MAP-1 is promoted from Draft to Approved.

## Test Environments

| Environment | How |
|---|---|
| Unit (`pytest`) | ROS2 package tests next to the package (`src/navigation`, `src/mapping`), pipeline tests in `tools/photogrammetry/tests/`; runs in CI |
| Smoke | `scripts/smoke_test.sh` in the platform Docker image — extend the node list with `survey_recorder_node` (WP-2) |
| SITL | `sim-test` agent via the workflow's built-in `run_simulation` stage — scenario + pass criteria written into the submitted plan text, **not** a plan step (the `simulation` queue serves no other activity; see the implementation plan's harness constraints) |
| Ground station | Reference ground-station hardware defined in DES-005; pipeline runs are automated but hardware-bound |
| Field | Manual demonstration flight; results recorded in `docs/reports/` |

## Reference test fixtures

- **Reference polygon:** a 100 m × 80 m rectangle plus one convex 6-vertex
  polygon, checked into the test fixtures with WP-1 (used by MAP-1, MAP-5,
  and the WP-4 end-to-end run so results are comparable across WPs).
- **Reference sample dataset:** small (≈ 20-frame) hand-built dataset in the
  DES-004 format, checked in with WP-3, so the pipeline is testable without
  flying.

## Exit Criteria

- All **Test**-method rows implemented with `Verifies:` markers and passing.
- MAP-1 rows green under `python scripts/check_traceability.py --strict`
  (safety-critical must not remain `planned`).
- Gap report `docs/reports/gap_CAP-001.md` reads 14/14 present.
- STK-1 Demonstration row has a dated report in `docs/reports/`.
