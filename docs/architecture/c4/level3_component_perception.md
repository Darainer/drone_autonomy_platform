<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/generate_c4.py -->
# C4 Level 3 — Component View: `src/perception`

![C4 Level 3 — Component View: `src/perception`](level3_component_perception.svg)

Diagram source: [`level3_component_perception.puml`](level3_component_perception.puml) (C4-PlantUML, rendered with Graphviz).

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
