<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/check_architecture_gap.py -->

# Capability Gap Report — CAP-001: Visual Photogrammetry / Survey Mapping

Target spec: `docs/architecture/target/CAP-001-photogrammetry.yaml` · Stakeholder requirements: STK-1

**18 / 18** target elements present — **capability complete**

## Containers

| Target element | Requirement | Status | Detail |
|---|---|---|---|
| autonomy_node | MAP-6 | ✅ present | ROS2 node parsed from src/ |
| navigation_node | MAP-1 | ✅ present | ROS2 node parsed from src/ |
| oakd | — | ✅ present | registered in EXTERNAL_SYSTEMS |
| mavros | — | ✅ present | registered in EXTERNAL_SYSTEMS |
| survey_recorder_node | MAP-2 | ✅ present | ROS2 node parsed from src/ |
| photogrammetry_pipeline | MAP-4 | ✅ present | `tools/photogrammetry` exists |

## Flows

| Target element | Requirement | Status | Detail |
|---|---|---|---|
| autonomy_node → navigation_node | MAP-6 | ✅ present | `/mission` |
| navigation_node → control_node | MAP-1 | ✅ present | `/trajectory` |
| oakd → survey_recorder_node | MAP-2 | ✅ present | `/oak/rgb/image_raw` |
| mavros → survey_recorder_node | MAP-2 | ✅ present | `/mavros/local_position/pose` |
| mavros → survey_recorder_node | MAP-2 | ✅ present | `/mavros/global_position/global` |
| autonomy_node → survey_recorder_node | MAP-2 | ✅ present | `/mission` |
| autonomy_node → survey_recorder_node | MAP-2 | ✅ present | `/mission_status` |

## Behaviors

| Target element | Requirement | Status | Detail |
|---|---|---|---|
| Survey mission type in mission manager | MAP-6 | ✅ present | `Implements: MAP-6` in src/autonomy/include/autonomy/autonomy_node.hpp, src/autonomy/include/autonomy/survey_validation.hpp, src/autonomy/src/autonomy_node.cpp |
| Coverage-pattern trajectory generator in navigation | MAP-1 | ✅ present | `Implements: MAP-1` in src/navigation/include/navigation/survey_planner.hpp, src/navigation/src/survey_planner.cpp |
| Dataset offload format documented and implemented | MAP-3 | ✅ present | `Implements: MAP-3` in src/mapping/include/mapping/dataset_writer.hpp |
| Onboard post-flight consistency check | MAP-7 | ✅ present | `Implements: MAP-7` in tools/photogrammetry/run_pipeline.py |
| Reconstruction pipeline executable on companion | MAP-8 | ✅ present | `Implements: MAP-8` in tools/photogrammetry/run_pipeline.py |
