<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/check_traceability.py -->

# Requirements Traceability Matrix

| UID | Title | Status | Design/Docs | Code (Implements) | Tests (Verifies) | Coverage |
|---|---|---|---|---|---|---|
| COMP-1 | Camera Capture Latency | Approved | docs/architecture/latency_requirements.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| COMP-2 | Image Processing Latency | Approved | docs/architecture/latency_requirements.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| COMP-3 | Object Detection Latency | Approved | docs/architecture/latency_requirements.md<br>docs/architecture/perception_architecture.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| COMP-4 | Object Tracking Latency | Approved | docs/architecture/latency_requirements.md<br>docs/architecture/perception_architecture.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| COMP-5 | Decision Making Latency | Approved | docs/architecture/latency_requirements.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| COMP-6 ⚠ | Control Command Generation Latency | Approved | docs/architecture/latency_requirements.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| COMP-7 ⚠ | Flight Controller Processing Latency | Approved | docs/architecture/latency_requirements.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| COMP-8 | Data Link Uplink Latency | Approved | docs/architecture/latency_requirements.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| COMP-9 | Data Link Downlink Latency | Approved | docs/architecture/latency_requirements.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| E2E-1 | Sensor to GCS Display Latency | Approved | docs/architecture/latency_requirements.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| E2E-2 ⚠ | Sensor to Action Latency (Threat Response) | Approved | docs/architecture/latency_requirements.md<br>docs/architecture/perception_architecture.md<br>docs/design/DES-001-topic-remap-wiring.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| E2E-3 | Sensor to Action Latency (Target Tracking) | Approved | docs/architecture/latency_requirements.md<br>docs/design/DES-001-topic-remap-wiring.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| PLAT-1 | Sensor Input to Perception Output | Approved | docs/architecture/latency_requirements.md<br>docs/architecture/perception_architecture.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| PLAT-2 | Perception Output to Control Command | Approved | docs/architecture/latency_requirements.md<br>docs/design/DES-001-topic-remap-wiring.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| PLAT-3 | Total Onboard Processing | Approved | docs/architecture/latency_requirements.md<br>docs/test_plans/TP-001-latency-and-safety.md | — | — | 🟡 planned |
| SAF-1 ⚠ | Critical Battery Return-to-Launch | Approved | docs/design/DES-001-topic-remap-wiring.md<br>docs/test_plans/TP-001-latency-and-safety.md | src/safety/src/battery_monitor.cpp | — | 🟡 planned |
| SAF-2 ⚠ | Low Battery Warning | Approved | docs/design/DES-001-topic-remap-wiring.md<br>docs/test_plans/TP-001-latency-and-safety.md | src/safety/src/battery_monitor.cpp | — | 🟡 planned |

## Summary

- Requirements: **17**
- Verified by test: **0**
- Verification planned (test plan only): **17**
- Uncovered: **0**

⚠ = tagged `safety-critical` (DO-178C review scope, see docs/standards/do_178c_context.md)

Linkage markers: `Implements: <UID>` in source, `Verifies: <UID>` in tests, bare UID mentions in design docs and test plans.
