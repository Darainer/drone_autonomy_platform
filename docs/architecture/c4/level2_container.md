<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/generate_c4.py -->
# C4 Level 2 — Container View (ROS2 Nodes)

Every deployable ROS2 node in `src/`, grouped by package, with topic and
service flows extracted from the source. Edges labelled *needs remap* match
by topic basename only — see `topics.md` for details.

```mermaid
C4Container
    title Container View — ROS2 nodes of the Drone Autonomy Platform

    System_Ext(mavros, "MAVROS / PX4 Autopilot", "MAVLink bridge to the PX4 flight controller")
    System_Ext(oakd, "OAK-D Camera Driver", "depthai-ros driver publishing RGB + stereo depth")

    System_Boundary(platform, "Drone Autonomy Platform") {
        Container_Boundary(autonomy_pkg, "src/autonomy") {
            Container(autonomy_node, "autonomy_node", "ROS2 / C++", "src/autonomy/src/autonomy_node.cpp")
        }
        Container_Boundary(communication_pkg, "src/communication") {
            Container(communication_node, "communication_node", "ROS2 / C++", "src/communication/src/communication_node.cpp")
        }
        Container_Boundary(control_pkg, "src/control") {
            Container(control_node, "control_node", "ROS2 / C++", "src/control/src/control_node.cpp")
        }
        Container_Boundary(navigation_pkg, "src/navigation") {
            Container(navigation_node, "navigation_node", "ROS2 / C++", "src/navigation/src/navigation_node.cpp")
        }
        Container_Boundary(perception_pkg, "src/perception") {
            Container(detection_visualizer, "detection_visualizer", "ROS2 / Python", "src/perception/src/detection_visualizer.py")
            Container(perception_node, "perception_node", "ROS2 / C++", "src/perception/src/perception_node.cpp")
            Container(rfdetr_node, "rfdetr_node", "ROS2 / Python", "src/perception/src/rfdetr_node.py")
        }
        Container_Boundary(safety_pkg, "src/safety") {
            Container(battery_monitor, "battery_monitor", "ROS2 / C++", "src/safety/src/battery_monitor.cpp")
            Container(safety_node, "safety_node", "ROS2 / C++", "src/safety/src/safety_node.cpp")
        }
    }

    Rel(autonomy_node, navigation_node, "mission (needs remap)", "drone_autonomy_msgs/Mission")
    Rel(control_node, communication_node, "attitude_command (needs remap)", "drone_autonomy_msgs/AttitudeCommand")
    Rel(rfdetr_node, detection_visualizer, "/detections", "Detection2DArray")
    Rel(rfdetr_node, perception_node, "/detections", "Detection2DArray")
    Rel(mavros, battery_monitor, "/mavros/battery", "sensor_msgs/BatteryState")
    Rel(mavros, communication_node, "/mavros/state", "mavros_msgs/State")
    Rel(mavros, safety_node, "/mavros/state", "mavros_msgs/State")
    Rel(navigation_node, control_node, "trajectory (needs remap)", "drone_autonomy_msgs/Trajectory")
    Rel(oakd, detection_visualizer, "/oak/rgb/image_raw", "Image")
    Rel(oakd, perception_node, "/oak/rgb/image_raw", "sensor_msgs/Image")
    Rel(oakd, rfdetr_node, "/oak/rgb/image_raw", "Image")
    Rel(oakd, perception_node, "/oak/stereo/image_raw", "sensor_msgs/Image")
    Rel(battery_monitor, mavros, "srv: /mavros/set_mode", "mavros_msgs/SetMode")

    UpdateLayoutConfig($c4ShapeInRow="3", $c4BoundaryInRow="2")
```
