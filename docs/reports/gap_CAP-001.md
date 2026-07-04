<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/check_architecture_gap.py -->

# Capability Gap Report — CAP-001: Visual Photogrammetry / Survey Mapping

Target spec: `docs/architecture/target/CAP-001-photogrammetry.yaml` · Stakeholder requirements: STK-1

**6 / 18** target elements present — **12 gap(s) remain**

## Containers

| Target element | Requirement | Status | Detail |
|---|---|---|---|
| autonomy_node | MAP-6 | ✅ present | ROS2 node parsed from src/ |
| navigation_node | MAP-1 | ✅ present | ROS2 node parsed from src/ |
| oakd | — | ✅ present | registered in EXTERNAL_SYSTEMS |
| mavros | — | ✅ present | registered in EXTERNAL_SYSTEMS |
| survey_recorder_node | MAP-2 | ❌ missing | no such node in src/ |
| photogrammetry_pipeline | MAP-4 | ❌ missing | `tools/photogrammetry` does not exist |

## Flows

| Target element | Requirement | Status | Detail |
|---|---|---|---|
| autonomy_node → navigation_node | MAP-6 | ✅ present | `/mission` |
| navigation_node → control_node | MAP-1 | ✅ present | `/trajectory` |
| oakd → survey_recorder_node | MAP-2 | ❌ missing | `/oak/rgb/image_raw` — no such edge in current topic graph |
| mavros → survey_recorder_node | MAP-2 | ❌ missing | `/mavros/local_position/pose` — no such edge in current topic graph |
| mavros → survey_recorder_node | MAP-2 | ❌ missing | `/mavros/global_position/global` — no such edge in current topic graph |
| autonomy_node → survey_recorder_node | MAP-2 | ❌ missing | `/mission` — no such edge in current topic graph |
| autonomy_node → survey_recorder_node | MAP-2 | ❌ missing | `/mission_status` — no such edge in current topic graph |

## Behaviors

| Target element | Requirement | Status | Detail |
|---|---|---|---|
| Survey mission type in mission manager | MAP-6 | ❌ missing | no `Implements: MAP-6` marker under src/ |
| Coverage-pattern trajectory generator in navigation | MAP-1 | ❌ missing | no `Implements: MAP-1` marker under src/ |
| Dataset offload format documented and implemented | MAP-3 | ❌ missing | no `Implements: MAP-3` marker under src/ |
| Onboard post-flight consistency check | MAP-7 | ❌ missing | no `Implements: MAP-7` marker under src/ |
| Reconstruction pipeline executable on companion | MAP-8 | ❌ missing | no `Implements: MAP-8` marker under src/ |

## Gap list (implementation handoff input)

- [ ] survey_recorder_node (MAP-2)
- [ ] photogrammetry_pipeline (MAP-4)
- [ ] oakd → survey_recorder_node (MAP-2)
- [ ] mavros → survey_recorder_node (MAP-2)
- [ ] mavros → survey_recorder_node (MAP-2)
- [ ] autonomy_node → survey_recorder_node (MAP-2)
- [ ] autonomy_node → survey_recorder_node (MAP-2)
- [ ] Survey mission type in mission manager (MAP-6)
- [ ] Coverage-pattern trajectory generator in navigation (MAP-1)
- [ ] Dataset offload format documented and implemented (MAP-3)
- [ ] Onboard post-flight consistency check (MAP-7)
- [ ] Reconstruction pipeline executable on companion (MAP-8)

Turn gaps into work: see the `capability` skill — each gap cluster gets a design doc (`design` skill) and an agent work package.
