<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/generate_c4.py -->

# Topic & Service Inventory

| Topic | Type | Publishers | Subscribers | Status |
|---|---|---|---|---|
| `/attitude_command` | `drone_autonomy_msgs/AttitudeCommand` | control_node | communication_node | connected |
| `/detections` | `Detection2DArray` | rfdetr_node | detection_visualizer, perception_node | connected |
| `/mavros/battery` | `sensor_msgs/BatteryState` | — | battery_monitor | external (MAVROS / PX4 Autopilot) |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | — | survey_recorder_node | external (MAVROS / PX4 Autopilot) |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | — | survey_recorder_node | external (MAVROS / PX4 Autopilot) |
| `/mavros/state` | `mavros_msgs/State` | — | communication_node, safety_node | external (MAVROS / PX4 Autopilot) |
| `/mission` | `drone_autonomy_msgs/Mission` | autonomy_node | survey_recorder_node, navigation_node | connected |
| `/mission_status` | `drone_autonomy_msgs/MissionStatus` | autonomy_node | survey_recorder_node | connected |
| `/oak/rgb/camera_info` | `sensor_msgs/CameraInfo` | — | survey_recorder_node | external (OAK-D Camera Driver) |
| `/oak/rgb/image_raw` | `sensor_msgs/Image` | — | survey_recorder_node, detection_visualizer, perception_node, rfdetr_node | external (OAK-D Camera Driver) |
| `/oak/stereo/image_raw` | `sensor_msgs/Image` | — | perception_node | external (OAK-D Camera Driver) |
| `/perception_node/sensor_data` | `drone_autonomy_msgs/SensorData` | perception_node | — | dangling (no subscriber) |
| `/survey_request` | `drone_autonomy_msgs/Mission` | — | autonomy_node | dangling (no publisher) |
| `/trajectory` | `drone_autonomy_msgs/Trajectory` | navigation_node | control_node | connected |
| `/visualization/detection_overlay` | `Image` | detection_visualizer | — | external (Operator Visualization) |
| `/mavros/set_mode (service)` | `mavros_msgs/SetMode` | battery_monitor (client) | — | external (MAVROS / PX4 Autopilot) |

## Dangling interfaces

- `/perception_node/sensor_data` — dangling (no subscriber)
- `/survey_request` — dangling (no publisher)
