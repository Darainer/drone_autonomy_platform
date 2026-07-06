<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/check_traceability.py -->

# Requirements Traceability Matrix

| UID | Title | Status | Design/Docs | Code (Implements) | Tests (Verifies) | Coverage |
|---|---|---|---|---|---|---|
| AUT-1 | Mission Decisions Use Fused Perception Output | Draft | docs/design/DES-002-sensor-data-consumption.md | — | — | ❌ uncovered |
| COMP-1 | Camera Capture Latency | Approved | docs/architecture/latency_requirements.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| COMP-2 | Image Processing Latency | Approved | docs/architecture/latency_requirements.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| COMP-3 | Object Detection Latency | Approved | docs/architecture/latency_requirements.md<br>docs/architecture/perception_architecture.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| COMP-4 | Object Tracking Latency | Approved | docs/architecture/latency_requirements.md<br>docs/architecture/perception_architecture.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| COMP-5 | Decision Making Latency | Approved | docs/architecture/latency_requirements.md<br>docs/design/DES-002-sensor-data-consumption.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| COMP-6 ⚠ | Control Command Generation Latency | Approved | docs/architecture/latency_requirements.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| COMP-7 ⚠ | Flight Controller Processing Latency | Approved | docs/architecture/latency_requirements.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| COMP-8 | Data Link Uplink Latency | Approved | docs/architecture/latency_requirements.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| COMP-9 | Data Link Downlink Latency | Approved | docs/architecture/latency_requirements.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| E2E-1 | Sensor to GCS Display Latency | Approved | docs/architecture/latency_requirements.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| E2E-2 ⚠ | Sensor to Action Latency (Obstacle Avoidance) | Approved | docs/architecture/latency_requirements.md<br>docs/architecture/perception_architecture.md<br>docs/design/DES-001-topic-remap-wiring.md<br>docs/design/DES-002-sensor-data-consumption.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| E2E-3 | Sensor to Action Latency (Feature Tracking) | Approved | docs/architecture/latency_requirements.md<br>docs/design/DES-001-topic-remap-wiring.md<br>docs/design/DES-002-sensor-data-consumption.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| MAP-1 ⚠ | Survey Trajectory Generation | Draft | docs/architecture/target/CAP-001-photogrammetry.yaml<br>docs/capabilities/CAP-001-implementation-plan.md<br>docs/capabilities/CAP-001-photogrammetry.md<br>docs/capabilities/CAP-001-remaining-work.md<br>docs/design/DES-003-survey-mission-coverage-trajectory.md<br>docs/test_plans/TP-002-survey-mapping.md | src/navigation/include/navigation/survey_planner.hpp<br>src/navigation/src/survey_planner.cpp | src/navigation/tests/survey_planner_test.cpp | ✅ verified |
| MAP-2 | Synchronized Survey Data Recording | Draft | docs/architecture/target/CAP-001-photogrammetry.yaml<br>docs/capabilities/CAP-001-implementation-plan.md<br>docs/capabilities/CAP-001-photogrammetry.md<br>docs/design/DES-004-survey-dataset-recording.md<br>docs/test_plans/TP-002-survey-mapping.md | src/mapping/src/survey_recorder_node.cpp | src/mapping/tests/test_survey_recorder_replay.py | ✅ verified |
| MAP-3 | Survey Dataset Offload | Draft | docs/architecture/target/CAP-001-photogrammetry.yaml<br>docs/capabilities/CAP-001-implementation-plan.md<br>docs/capabilities/CAP-001-photogrammetry.md<br>docs/design/DES-004-survey-dataset-recording.md<br>docs/design/DES-005-photogrammetry-pipeline.md<br>docs/test_plans/TP-002-survey-mapping.md | src/mapping/include/mapping/dataset_writer.hpp | src/mapping/tests/dataset_writer_test.cpp<br>tools/photogrammetry/tests/test_dataset.py<br>tools/photogrammetry/tests/test_verify_dataset.py | ✅ verified |
| MAP-4 | Post-Flight 3D Reconstruction | Draft | docs/architecture/target/CAP-001-photogrammetry.yaml<br>docs/capabilities/CAP-001-implementation-plan.md<br>docs/capabilities/CAP-001-photogrammetry.md<br>docs/design/DES-005-photogrammetry-pipeline.md<br>docs/test_plans/TP-002-survey-mapping.md | tools/photogrammetry/photogrammetry/odm_runner.py<br>tools/photogrammetry/run_pipeline.py | tools/photogrammetry/tests/test_odm_runner.py<br>tools/photogrammetry/tests/test_run_pipeline_full.py | ✅ verified |
| MAP-5 | Reconstruction Completeness | Draft | docs/capabilities/CAP-001-implementation-plan.md<br>docs/capabilities/CAP-001-photogrammetry.md<br>docs/design/DES-005-photogrammetry-pipeline.md<br>docs/test_plans/TP-002-survey-mapping.md | tools/photogrammetry/photogrammetry/coverage.py | tools/photogrammetry/tests/test_coverage.py | ✅ verified |
| MAP-6 | Survey Mission Type | Draft | docs/architecture/target/CAP-001-photogrammetry.yaml<br>docs/capabilities/CAP-001-implementation-plan.md<br>docs/capabilities/CAP-001-photogrammetry.md<br>docs/design/DES-003-survey-mission-coverage-trajectory.md<br>docs/test_plans/TP-002-survey-mapping.md | src/autonomy/include/autonomy/autonomy_node.hpp<br>src/autonomy/include/autonomy/survey_validation.hpp<br>src/autonomy/src/autonomy_node.cpp | src/autonomy/test/test_survey_dispatch.cpp | ✅ verified |
| MAP-7 | Onboard Post-Flight Consistency Check | Draft | docs/architecture/target/CAP-001-photogrammetry.yaml<br>docs/capabilities/CAP-001-implementation-plan.md<br>docs/capabilities/CAP-001-photogrammetry.md<br>docs/capabilities/CAP-001-remaining-work.md<br>docs/design/DES-004-survey-dataset-recording.md<br>docs/design/DES-005-photogrammetry-pipeline.md<br>docs/test_plans/TP-002-survey-mapping.md | tools/photogrammetry/run_pipeline.py | — | 🟡 planned |
| MAP-8 | Onboard Reconstruction Execution | Draft | docs/architecture/target/CAP-001-photogrammetry.yaml<br>docs/capabilities/CAP-001-implementation-plan.md<br>docs/capabilities/CAP-001-photogrammetry.md<br>docs/capabilities/CAP-001-remaining-work.md<br>docs/design/DES-005-photogrammetry-pipeline.md<br>docs/test_plans/TP-002-survey-mapping.md | tools/photogrammetry/run_pipeline.py | — | 🟡 planned |
| PER-1 | Fused Sensor Data Publication | Draft | docs/design/DES-002-sensor-data-consumption.md | — | — | ❌ uncovered |
| PLAT-1 | Sensor Input to Perception Output | Approved | docs/architecture/latency_requirements.md<br>docs/architecture/perception_architecture.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| PLAT-2 | Perception Output to Control Command | Approved | docs/architecture/latency_requirements.md<br>docs/design/DES-001-topic-remap-wiring.md<br>docs/design/DES-002-sensor-data-consumption.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| PLAT-3 | Total Onboard Processing | Approved | docs/architecture/latency_requirements.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| SAF-1 ⚠ | Critical Battery Return-to-Launch | Approved | docs/design/DES-001-topic-remap-wiring.md<br>docs/test_plans/TP-001-latency-and-safety.md | src/safety/src/battery_monitor.cpp | — | 🟡 planned |
| SAF-2 ⚠ | Low Battery Warning | Approved | docs/design/DES-001-topic-remap-wiring.md<br>docs/test_plans/TP-001-latency-and-safety.md | src/safety/src/battery_monitor.cpp | — | 🟡 planned |
| STK-1 | Post-Flight 3D Survey Mapping | Draft | docs/architecture/target/CAP-001-photogrammetry.yaml<br>docs/capabilities/CAP-001-implementation-plan.md<br>docs/capabilities/CAP-001-photogrammetry.md<br>docs/capabilities/CAP-001-remaining-work.md<br>docs/design/DES-004-survey-dataset-recording.md<br>docs/design/DES-005-photogrammetry-pipeline.md<br>docs/test_plans/TP-002-survey-mapping.md | — | — | 🟡 planned |

## Summary

- Requirements: **28**
- Verified by test: **6**
- Verification planned (test plan only): **20**
- Uncovered: **2**

⚠ = tagged `safety-critical` (DO-178C review scope, see docs/standards/do_178c_context.md)

Linkage markers: `Implements: <UID>` in source, `Verifies: <UID>` in tests, bare UID mentions in design docs and test plans.
