<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/generate_c4.py -->
# C4 Level 3 — Component View: `src/mapping`

```mermaid
C4Component
    title Component View — src/mapping

    Container(autonomy_node, "autonomy_node", "ROS2 node", "src/autonomy")
    System_Ext(mavros, "MAVROS / PX4 Autopilot", "MAVLink bridge to the PX4 flight controller")
    System_Ext(oakd, "OAK-D Camera Driver", "depthai-ros driver publishing RGB + stereo depth")

    Container_Boundary(mapping, "src/mapping") {
        Component(survey_recorder_node, "survey_recorder_node", "ROS2 node (C++)", "0 pub / 6 sub")
    }

    Rel(mavros, survey_recorder_node, "/mavros/global_position/global", "sensor_msgs/NavSatFix")
    Rel(mavros, survey_recorder_node, "/mavros/local_position/pose", "geometry_msgs/PoseStamped")
    Rel(autonomy_node, survey_recorder_node, "/mission", "drone_autonomy_msgs/Mission")
    Rel(autonomy_node, survey_recorder_node, "/mission_status", "drone_autonomy_msgs/MissionStatus")
    Rel(oakd, survey_recorder_node, "/oak/rgb/camera_info", "sensor_msgs/CameraInfo")
    Rel(oakd, survey_recorder_node, "/oak/rgb/image_raw", "sensor_msgs/Image")
```

## Interfaces

| Node | Direction | Topic / Service | Type |
|---|---|---|---|
| `survey_recorder_node` | subscribes | `/oak/rgb/camera_info` | `sensor_msgs/CameraInfo` |
| `survey_recorder_node` | subscribes | `/mavros/global_position/global` | `sensor_msgs/NavSatFix` |
| `survey_recorder_node` | subscribes | `/mission` | `drone_autonomy_msgs/Mission` |
| `survey_recorder_node` | subscribes | `/mission_status` | `drone_autonomy_msgs/MissionStatus` |
| `survey_recorder_node` | subscribes | `/oak/rgb/image_raw` | `sensor_msgs/Image` |
| `survey_recorder_node` | subscribes | `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` |
