<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/generate_c4.py -->
# C4 Level 3 — Component View: `src/perception`

```mermaid
C4Component
    title Component View — src/perception

    System_Ext(oakd, "OAK-D Camera Driver", "depthai-ros driver publishing RGB + stereo depth")

    Container_Boundary(perception, "src/perception") {
        Component(detection_visualizer, "detection_visualizer", "ROS2 node (Python)", "1 pub / 2 sub")
        Component(perception_node, "perception_node", "ROS2 node (C++)", "1 pub / 3 sub")
        Component(rfdetr_node, "rfdetr_node", "ROS2 node (Python)", "1 pub / 1 sub")
    }

    Rel(rfdetr_node, detection_visualizer, "/detections", "Detection2DArray")
    Rel(rfdetr_node, perception_node, "/detections", "Detection2DArray")
    Rel(oakd, detection_visualizer, "/oak/rgb/image_raw", "Image")
    Rel(oakd, perception_node, "/oak/rgb/image_raw", "sensor_msgs/Image")
    Rel(oakd, rfdetr_node, "/oak/rgb/image_raw", "Image")
    Rel(oakd, perception_node, "/oak/stereo/image_raw", "sensor_msgs/Image")
```

## Interfaces

| Node | Direction | Topic / Service | Type |
|---|---|---|---|
| `detection_visualizer` | publishes | `/visualization/detection_overlay` | `Image` |
| `detection_visualizer` | subscribes | `/oak/rgb/image_raw` | `Image` |
| `detection_visualizer` | subscribes | `/detections` | `Detection2DArray` |
| `perception_node` | publishes | `/perception_node/sensor_data` | `drone_autonomy_msgs/SensorData` |
| `perception_node` | subscribes | `/oak/rgb/image_raw` | `sensor_msgs/Image` |
| `perception_node` | subscribes | `/oak/stereo/image_raw` | `sensor_msgs/Image` |
| `perception_node` | subscribes | `/detections` | `vision_msgs/Detection2DArray` |
| `rfdetr_node` | publishes | `/detections` | `Detection2DArray` |
| `rfdetr_node` | subscribes | `/oak/rgb/image_raw` | `Image` |

## Parser notes

- topic `/oak/rgb/image_raw` comes from parameter default
