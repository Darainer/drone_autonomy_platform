<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/generate_c4.py -->

# Topic & Service Inventory

| Topic | Type | Publishers | Subscribers | Status |
|---|---|---|---|---|
| `/autonomy_node/mission` | `drone_autonomy_msgs/Mission` | autonomy_node | — | needs remap |
| `/communication_node/attitude_command` | `drone_autonomy_msgs/AttitudeCommand` | — | communication_node | needs remap |
| `/control_node/attitude_command` | `drone_autonomy_msgs/AttitudeCommand` | control_node | — | needs remap |
| `/control_node/trajectory` | `drone_autonomy_msgs/Trajectory` | — | control_node | needs remap |
| `/detections` | `Detection2DArray` | rfdetr_node | detection_visualizer, perception_node | connected |
| `/mavros/battery` | `sensor_msgs/BatteryState` | — | battery_monitor | external (MAVROS / PX4 Autopilot) |
| `/mavros/state` | `mavros_msgs/State` | — | communication_node, safety_node | external (MAVROS / PX4 Autopilot) |
| `/navigation_node/mission` | `drone_autonomy_msgs/Mission` | — | navigation_node | needs remap |
| `/navigation_node/trajectory` | `drone_autonomy_msgs/Trajectory` | navigation_node | — | needs remap |
| `/oak/rgb/image_raw` | `Image` | — | detection_visualizer, perception_node, rfdetr_node | external (OAK-D Camera Driver) |
| `/oak/stereo/image_raw` | `sensor_msgs/Image` | — | perception_node | external (OAK-D Camera Driver) |
| `/perception_node/sensor_data` | `drone_autonomy_msgs/SensorData` | perception_node | — | dangling (no subscriber) |
| `/visualization/detection_overlay` | `Image` | detection_visualizer | — | dangling (no subscriber) |
| `/mavros/set_mode (service)` | `mavros_msgs/SetMode` | battery_monitor (client) | — | external (MAVROS / PX4 Autopilot) |

## ⚠ Remap warnings

- `autonomy_node` publishes `/autonomy_node/mission` but `navigation_node` subscribes `/navigation_node/mission` — same basename `mission`; these only connect if remapped in a launch file.
- `control_node` publishes `/control_node/attitude_command` but `communication_node` subscribes `/communication_node/attitude_command` — same basename `attitude_command`; these only connect if remapped in a launch file.
- `navigation_node` publishes `/navigation_node/trajectory` but `control_node` subscribes `/control_node/trajectory` — same basename `trajectory`; these only connect if remapped in a launch file.

## Dangling interfaces

- `/perception_node/sensor_data` — dangling (no subscriber)
- `/visualization/detection_overlay` — dangling (no subscriber)
