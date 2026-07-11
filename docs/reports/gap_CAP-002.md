<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/check_architecture_gap.py -->

# Capability Gap Report — CAP-002: Pre-Mow Field Clearance Surveillance

Target spec: `docs/architecture/target/CAP-002-premow-clearance.yaml` · Stakeholder requirements: STK-2

**9 / 37** target elements present — **28 gap(s) remain**

## Containers

| Target element | Requirement | Status | Detail |
|---|---|---|---|
| autonomy_node | CLR-1 | ✅ present | ROS2 node parsed from src/ |
| navigation_node | CLR-8 | ✅ present | ROS2 node parsed from src/ |
| rfdetr_node | CLR-2 | ✅ present | ROS2 node parsed from src/ |
| communication_node | CLR-7 | ✅ present | ROS2 node parsed from src/ |
| oakd | — | ✅ present | registered in EXTERNAL_SYSTEMS |
| mavros | — | ✅ present | registered in EXTERNAL_SYSTEMS |
| thermal_cam | CLR-11 | ❌ missing | not in generate_c4.EXTERNAL_SYSTEMS |
| tracker_node | CLR-3 | ❌ missing | no such node in src/ |
| thermal_detector_node | CLR-12 | ❌ missing | no such node in src/ |
| clearance_recorder_node | CLR-5 | ❌ missing | no such node in src/ |
| clearance_report_pipeline | CLR-6 | ❌ missing | `tools/clearance_report` does not exist |
| clearance_model_pipeline | CLR-9 | ❌ missing | `tools/clearance_model` does not exist |

## Flows

| Target element | Requirement | Status | Detail |
|---|---|---|---|
| autonomy_node → navigation_node | CLR-1 | ✅ present | `/mission` |
| navigation_node → control_node | CLR-8 | ✅ present | `/trajectory` |
| oakd → rfdetr_node | CLR-2 | ✅ present | `/oak/rgb/image_raw` |
| rfdetr_node → tracker_node | CLR-3 | ❌ missing | `/detections` — no such edge in current topic graph |
| thermal_cam → thermal_detector_node | CLR-12 | ❌ missing | `/thermal/image_raw` — no such edge in current topic graph |
| thermal_detector_node → tracker_node | CLR-12 | ❌ missing | `/detections` — no such edge in current topic graph |
| thermal_cam → clearance_recorder_node | CLR-5 | ❌ missing | `/thermal/image_raw` — no such edge in current topic graph |
| tracker_node → clearance_recorder_node | CLR-4 | ❌ missing | `/tracked_objects` — no such edge in current topic graph |
| oakd → clearance_recorder_node | CLR-5 | ❌ missing | `/oak/rgb/image_raw` — no such edge in current topic graph |
| oakd → clearance_recorder_node | CLR-4 | ❌ missing | `/oak/rgb/camera_info` — no such edge in current topic graph |
| mavros → clearance_recorder_node | CLR-4 | ❌ missing | `/mavros/local_position/pose` — no such edge in current topic graph |
| mavros → clearance_recorder_node | CLR-4 | ❌ missing | `/mavros/global_position/global` — no such edge in current topic graph |
| autonomy_node → clearance_recorder_node | CLR-5 | ❌ missing | `/mission` — no such edge in current topic graph |
| autonomy_node → clearance_recorder_node | CLR-5 | ❌ missing | `/mission_status` — no such edge in current topic graph |
| clearance_recorder_node → communication_node | CLR-7 | ❌ missing | `/findings` — no such edge in current topic graph |

## Behaviors

| Target element | Requirement | Status | Detail |
|---|---|---|---|
| Clearance mission type in mission manager | CLR-1 | ❌ missing | no `Implements: CLR-1` marker under src/ |
| Clearance-domain detection model deployed (label set + engine config) | CLR-2 | ❌ missing | no `Implements: CLR-2` marker under src/ |
| Surveillance (detection-GSD) coverage parameterization in navigation | CLR-8 | ❌ missing | no `Implements: CLR-8` marker under src/ |
| Finding confirmation maneuver (descend + native-crop re-inference) | CLR-10 | ❌ missing | no `Implements: CLR-10` marker under src/ |
| Dawn-sweep thermal detection meets E-THERM gate | CLR-12 | ❌ missing | no `Implements: CLR-12` marker under src/ |
| Finding geolocation (pixel -> WGS84, polygon containment) | CLR-4 | ❌ missing | no `Implements: CLR-4` marker under src/ |
| Evidence clip capture with pre-roll + finding records | CLR-5 | ❌ missing | no `Implements: CLR-5` marker under src/ |
| Per-area clearance report generation (verdicts, coverage holes) | CLR-6 | ❌ missing | no `Implements: CLR-6` marker under src/ |
| Report delivery to registered mobile device | CLR-7 | ❌ missing | no `Implements: CLR-7` marker under src/ |
| License-vetted dataset provenance manifest + labeling program | CLR-9 | ❌ missing | no `Implements: CLR-9` marker under src/ |

## Gap list (implementation handoff input)

- [ ] thermal_cam (CLR-11)
- [ ] tracker_node (CLR-3)
- [ ] thermal_detector_node (CLR-12)
- [ ] clearance_recorder_node (CLR-5)
- [ ] clearance_report_pipeline (CLR-6)
- [ ] clearance_model_pipeline (CLR-9)
- [ ] rfdetr_node → tracker_node (CLR-3)
- [ ] thermal_cam → thermal_detector_node (CLR-12)
- [ ] thermal_detector_node → tracker_node (CLR-12)
- [ ] thermal_cam → clearance_recorder_node (CLR-5)
- [ ] tracker_node → clearance_recorder_node (CLR-4)
- [ ] oakd → clearance_recorder_node (CLR-5)
- [ ] oakd → clearance_recorder_node (CLR-4)
- [ ] mavros → clearance_recorder_node (CLR-4)
- [ ] mavros → clearance_recorder_node (CLR-4)
- [ ] autonomy_node → clearance_recorder_node (CLR-5)
- [ ] autonomy_node → clearance_recorder_node (CLR-5)
- [ ] clearance_recorder_node → communication_node (CLR-7)
- [ ] Clearance mission type in mission manager (CLR-1)
- [ ] Clearance-domain detection model deployed (label set + engine config) (CLR-2)
- [ ] Surveillance (detection-GSD) coverage parameterization in navigation (CLR-8)
- [ ] Finding confirmation maneuver (descend + native-crop re-inference) (CLR-10)
- [ ] Dawn-sweep thermal detection meets E-THERM gate (CLR-12)
- [ ] Finding geolocation (pixel -> WGS84, polygon containment) (CLR-4)
- [ ] Evidence clip capture with pre-roll + finding records (CLR-5)
- [ ] Per-area clearance report generation (verdicts, coverage holes) (CLR-6)
- [ ] Report delivery to registered mobile device (CLR-7)
- [ ] License-vetted dataset provenance manifest + labeling program (CLR-9)

Turn gaps into work: see the `capability` skill — each gap cluster gets a design doc (`design` skill) and an agent work package.
