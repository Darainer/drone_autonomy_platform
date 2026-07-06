# TP-001 — Latency & Battery Failsafe Test Plan

**Status:** Draft
**Requirements document:** [docs/requirements/platform_requirements.sdoc](../requirements/platform_requirements.sdoc)

## Scope

Verification planning for the surveying latency budget (E2E-*, COMP-*, PLAT-*) and
the battery failsafe (SAF-*). Created with the `test-plan` skill; linkage is
checked by `scripts/check_traceability.py`.

## Requirements Coverage

| UID | Method | Environment | Test / Procedure | Status |
|---|---|---|---|---|
| SAF-1 | Test | SITL (`sim-test` queue) | Inject `sensor_msgs/BatteryState` at 14% on `/mavros/battery`; assert exactly one `SetMode(AUTO.RTL)` call | planned |
| SAF-2 | Test | SITL (`sim-test` queue) | Inject battery at 24%; assert warning log emitted, no mode change | planned |
| PLAT-1 | Test | Jetson HIL | Timestamp delta `/oak/rgb/image_raw` → `/detections`, p99 ≤ 150 ms over 1000 frames | planned |
| COMP-3 | Test | Jetson HIL | RF-DETR TensorRT engine benchmark, p99 inference ≤ 100 ms (`scripts/build_tensorrt_engine.sh` output) | planned |
| COMP-4 | Analysis | — | Tracking not yet implemented; budget verified at design time | planned |
| PLAT-2 | Test | SITL | Timestamp delta `/detections` → attitude command, p99 ≤ 30 ms | planned |
| PLAT-3 | Analysis | — | Derived: PLAT-1 + PLAT-2 measurements | planned |
| E2E-1 | Demonstration | Field test | Stopwatch capture-to-GCS-display over video downlink | planned |
| E2E-2 | Analysis | — | Budget roll-up of COMP-1..3, COMP-5..7 measurements | planned |
| E2E-3 | Analysis | — | Budget roll-up including COMP-4 | planned |
| COMP-1 | Inspection | — | OAK-D datasheet / driver timestamps | planned |
| COMP-2 | Test | Jetson HIL | Preprocess benchmark in `rfdetr_node.py` stats timer | planned |
| COMP-5 | Test | SITL | Mission manager decision timing | planned |
| COMP-6 | Test | SITL | Control command generation timing | planned |
| COMP-7 | Inspection | — | PX4 control loop documentation (≥ 250 Hz) | planned |
| COMP-8 | Test | Field test | Datalink RTT measurement | planned |
| COMP-9 | Test | Field test | Video downlink latency measurement | planned |

Methods: **Test** (automated, carries a `Verifies:` marker once implemented),
**Analysis**, **Inspection**, **Demonstration** (per DO-178C verification methods).

## Test Environments

| Environment | How |
|---|---|
| Unit (`pytest`) | `agents/tests/`, runs in CI |
| Smoke | `scripts/smoke_test.sh` in the platform Docker image |
| SITL | `sim-test` agent on the `simulation` queue |
| Jetson HIL / Field | Manual, results recorded in `docs/reports/` |

## Exit Criteria

- All **Test**-method rows implemented with `Verifies:` markers and passing.
- `python scripts/check_traceability.py --strict` passes for SAF-* rows
  (safety-critical requirements must not remain `planned`).
