# Perception Architecture: RF-DETR RGB Pipeline

## Overview

This document specifies the real-time object detection pipeline for surveying and agricultural autonomy applications using RF-DETR on NVIDIA Jetson Orin. The architecture prioritizes detection performance on the RGB image path with single-camera input, running as a custom TensorRT-accelerated ROS2 node.

**Primary Objectives:**
- **Detect and Avoid (DAA)**: A detect and avoid system is the primary reason to have onboard perception. The operational domain may contain a variety of flying and land-based moving objects, and the system must be able to dynamically react to obstructions along its planned flight path for safety.
- **Odometry**: The visual odometry is important to support mapping applications.

### Design Goals

| Goal | Target | Rationale |
|------|--------|-----------|
| Detection latency | ≤100ms | COMP-3 requirement from [latency_requirements.md](latency_requirements.md) |
| End-to-end perception | ≤150ms | PLAT-1 sensor-to-perception budget |
| Detection accuracy | ≥50% mAP | Reliable target identification at range |
| Tracking persistence | Stable IDs | Maintain lock on moving targets |

### Scope

- **In scope:** RGB camera pipeline, RF-DETR detection (custom TensorRT node), object tracking
- **Out of scope:** IR/thermal fusion (future), multi-camera batching (future), classical vision fallback (future)

---

## RF-DETR Model Architecture

### Why RF-DETR Over YOLO

| Criteria | RF-DETR | YOLOv8/11 |
|----------|---------|-----------|
| Architecture | Transformer (global context) | CNN (local features) |
| Post-processing | NMS-free (end-to-end) | Requires NMS |
| Small object detection | Superior (attention mechanism) | Good |
| License | Apache 2.0 | AGPL-3.0 (Ultralytics) / commercial |
| Jetson integration | Custom TensorRT ROS2 node (`rfdetr_node.py`) | No native Isaac ROS package either; would require the same custom-node approach |

RF-DETR (Roboflow's real-time detection transformer) provides strong detection of small, distant targets relevant to surveying and agricultural missions while remaining real-time on Jetson-class hardware. There is no native Isaac ROS package for RF-DETR (or RT-DETR); the deployed pipeline runs it through a hand-written TensorRT node rather than Isaac ROS composable nodes (see [History](#history-earlier-isaac-ros-design) below).

### Model Architecture

RF-DETR-Small is a DINOv2-backboned, real-time detection transformer: a vision-transformer backbone feeds a lightweight decoder that emits detections directly, without a separate NMS post-processing pass. The deployed node's docstring summarizes it as "NMS-free (end-to-end), Apache 2.0 licensed, and uses a DINOv2 backbone" (`src/perception/src/rfdetr_node.py`). For full architectural detail beyond what's needed to operate the deployed engine, see the [RF-DETR repository](https://github.com/roboflow/rf-detr).

```
┌───────────────────────────────────────────────────────────┐
│                    RF-DETR-Small (deployed)                │
├───────────────────────────────────────────────────────────┤
│                                                             │
│   ┌───────────────┐      ┌────────────────────────────┐   │
│   │  DINOv2        │────▶│  Detection decoder          │   │
│   │  backbone      │     │  (NMS-free, end-to-end)     │   │
│   └───────────────┘      └────────────────────────────┘   │
│         │                            │                     │
│         ▼                            ▼                     │
│   512×512 input               300 detections               │
│                                (boxes + 91-way logits)       │
│                                                             │
└───────────────────────────────────────────────────────────┘
```

---

## Model Selection

### Deployed Model: RF-DETR-Small

The platform deploys **RF-DETR-Small at 512×512 input resolution** — the model variant `rfdetr_node.py` loads via its TensorRT engine (default path `/home/dev/models/RF-DETR-SMALL.engine`). RF-DETR ships in several sizes (Nano/Small/Medium/Large per the Roboflow release); Small was selected as the best fit for Jetson Orin Nano's compute/thermal envelope while leaving headroom for the concurrent cuVSLAM (Isaac ROS visual odometry) pipeline.

We do not reproduce third-party benchmark numbers (mAP/FPS) here, since they vary by hardware, precision, and RF-DETR release version and would go stale. For current benchmarks, see the [RF-DETR repository](https://github.com/roboflow/rf-detr).

**Rationale:**
- NMS-free, end-to-end architecture gives a predictable latency bound (no variable-cost NMS pass)
- Strong small-object detection, useful for distant/aerial targets
- Smaller footprint (vs. Medium/Large) leaves compute headroom for the concurrent cuVSLAM mapping pipeline
- Apache 2.0 licensed

### Pretrained Weights Strategy

| Phase | Model | Dataset | Purpose |
|-------|-------|---------|---------|
| **Phase 1 (Current)** | RF-DETR-Small, COCO pretrained | COCO (80 classes) | Baseline deployment, validate pipeline |
| **Phase 2 (Planned)** | RF-DETR-Small fine-tuned | COCO + domain data | Improved general detection |
| **Phase 3 (Planned)** | Custom RF-DETR-Small | Mission-specific dataset | Surveying/agriculture-optimized classes |

#### Phase 1: Current Deployment

The deployed engine is exported from the stock `RFDETRSmall()` COCO-pretrained weights (via `scripts/export_rfdetr_onnx.py`) — no fine-tuning has been applied yet. `rfdetr_node.py` decodes against the standard 80-class COCO label set (plus a background class handled via softmax). See [Deployment Configuration](#deployment-configuration) for the export/build steps.

#### Phase 2-3: Fine-Tuning Roadmap

**Target Classes for Surveying/Agricultural Autonomy:**
- Agricultural machinery (tractors, harvesters, sprayers)
- Infrastructure (power lines, wind turbines, cell towers)
- Livestock (cattle, sheep, horses)
- Vehicles (cars, pickup trucks)
- Obstacles (trees, buildings, fences)

**Dataset Requirements:**
| Requirement | Specification |
|-------------|---------------|
| Minimum images | 10,000+ per class |
| Annotation format | COCO JSON |
| Image sources | Synthetic (Unity/Unreal), real drone footage, satellite imagery |
| Augmentations | Rotation, scale, weather, lighting, motion blur |
| Altitude variance | 50m - 500m AGL |
| Aspect angles | 0° - 90° (nadir to oblique) |

Full fine-tuning process, dataset plan, and training snippets are defined in [perception_finetuning_plan.md](perception_finetuning_plan.md).

---

## RGB Pipeline Architecture

### System Context

```
┌───────────────────────────────────────────────────────────────────────┐
│                        Perception Subsystem                           │
│                                                                        │
│  ┌─────────┐        ┌──────────────────────────┐      ┌────────────┐ │
│  │ Camera  │        │       rfdetr_node         │      │ perception_│ │
│  │ Driver  │───────▶│ preprocess → TensorRT     │─────▶│ node /     │ │
│  │(OAK-D)  │        │ inference → decode        │      │ detection_ │ │
│  │         │        │                            │      │ visualizer │ │
│  └─────────┘        └──────────────────────────┘      └────────────┘ │
│       │                          │                                    │
│     33ms                       ~28ms                                  │
│                                                                        │
│                  Total: ~61ms (within 150ms PLAT-1 budget)             │
└───────────────────────────────────────────────────────────────────────┘
```

### Node Graph (Deployed)

```
                    ┌─────────────────────────┐
                    │     Camera Driver       │
                    │  (depthai_ros_driver)   │
                    └───────────┬─────────────┘
                                │ sensor_msgs/Image
                                │ /oak/rgb/image_raw
                                ▼
                    ┌─────────────────────────┐
                    │       rfdetr_node        │
                    │  (src/perception, Python)│
                    │                          │
                    │  preprocess (512x512,    │
                    │   ImageNet normalize)    │
                    │  → TensorRT FP16 engine  │
                    │  → decode (softmax +     │
                    │    argmax + threshold)   │
                    └───────────┬─────────────┘
                                │ vision_msgs/Detection2DArray
                                │ /detections
                                ▼
                    ┌─────────────────────────┐
                    │   perception_node /      │
                    │   detection_visualizer   │
                    │   (drone_autonomy)       │
                    └─────────────────────────┘
```

#### History: Earlier Isaac ROS Design

An earlier revision of this pipeline was designed around Isaac ROS composable nodes (`isaac_ros_dnn_image_encoder` → `isaac_ros_rtdetr` → `isaac_ros_tensor_rt`) running RT-DETR. That design was never implemented; the deployed pipeline is the single custom `rfdetr_node.py` node described above, running RF-DETR-Small directly against a TensorRT engine via pycuda. Isaac ROS is still used elsewhere in the perception stack (`isaac_ros_visual_slam` / cuVSLAM for stereo visual odometry, see `src/perception/launch/vslam.launch.py`), just not in the detection path.

### Topic Specifications

| Topic | Message Type | Rate | Description |
|-------|--------------|------|--------------|
| `/oak/rgb/image_raw` | `sensor_msgs/Image` | 30 Hz | Raw RGB from OAK-D camera (sensor-data QoS) |
| `/detections` | `vision_msgs/Detection2DArray` | 30 Hz | Decoded bounding boxes + class hypotheses, published by `rfdetr_node` |

---

## Detection Node Configuration

### `rfdetr_node` Parameters

| Parameter | Default | Description |
|-----------|---------|--------------|
| `engine_path` | `/home/dev/models/RF-DETR-SMALL.engine` | TensorRT engine file |
| `confidence_threshold` | `0.5` | Minimum class-probability score to keep a detection |
| `image_topic` | `/oak/rgb/image_raw` | Input image topic |
| `max_detections` | `100` | Cap on published detections per frame (model emits up to 300 candidates) |

### In-Node Processing

1. **Preprocess** — resize to 512×512, ImageNet mean/std normalization, HWC→NCHW (`_preprocess`)
2. **Inference** — TensorRT FP16 execution via `pycuda`/`tensorrt` (`TensorRTEngine.infer`)
3. **Decode** — softmax over the 91-way logits (80 COCO classes + background), argmax for label/score, filter by `confidence_threshold`, keep top `max_detections` (`_postprocess`)

Launch entry points: `src/perception/launch/rfdetr.launch.py` (perception-only wrapper) and `src/perception/launch/full_stack.launch.py` (full stack including VSLAM); both start `rfdetr_node` alongside the OAK-D driver.

---

## Object Tracking

### Tracker Selection: ByteTrack

| Tracker | MOTA | Speed | Occlusion Handling | Selection |
|---------|------|-------|-------------------|-----------|
| **ByteTrack** | 80.3% | **30+ FPS** | Excellent (low-score association) | **Selected** |
| DeepSORT | 75.4% | 20 FPS | Good (ReID features) | Alternative |
| BoT-SORT | 80.5% | 25 FPS | Excellent | Future upgrade |

**ByteTrack Advantages:**
- Associates low-confidence detections (recovers occluded targets)
- No ReID network required (simpler, faster)
- Proven on UAV tracking datasets

> **Status:** Tracking is a planned integration — there is no tracker node in `src/perception` today. `rfdetr_node` publishes per-frame detections directly on `/detections`; `perception_node` and `detection_visualizer` consume that topic without track IDs. The design below is the target for a future `tracker_node`.

### Tracking Integration (Planned)

```python
# tracker_node.py (planned, not yet implemented)

from byte_tracker import BYTETracker

class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')

        self.tracker = BYTETracker(
            track_thresh=0.5,
            track_buffer=30,  # frames to keep lost tracks
            match_thresh=0.8,
            frame_rate=30,
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray, '/detections', self.detection_callback, 10)
        self.tracked_pub = self.create_publisher(
            Detection2DArray, '/tracked_objects', 10)

    def detection_callback(self, msg):
        detections = self.ros_to_numpy(msg)
        tracks = self.tracker.update(detections)
        tracked_msg = self.numpy_to_ros(tracks, msg.header)
        self.tracked_pub.publish(tracked_msg)
```

### Track State Output (Planned)

| Field | Type | Description |
|-------|------|--------------|
| `track_id` | uint32 | Persistent ID across frames |
| `bbox` | BoundingBox2D | Current bounding box |
| `class_id` | string | Object class |
| `confidence` | float | Detection confidence |
| `velocity` | Vector2 | Estimated pixel velocity |
| `age` | uint32 | Frames since track creation |
| `hits` | uint32 | Consecutive detection count |

---

## Latency Analysis

### Component Breakdown

| Component | Budget (ms) | Measured (ms) | Margin |
|-----------|-------------|---------------|--------|
| Camera capture | 30 | 33 (30 Hz) | -3 |
| Preprocess (rfdetr_node: resize + normalize) | 25 | 18 | +7 |
| **TensorRT inference** | **50** | **23** | **+27** |
| Decode (rfdetr_node: softmax + argmax + threshold) | 5 | 2 | +3 |
| ByteTrack (planned, not yet implemented) | 50 | — | — |
| **Total vs. PLAT-1 requirement** | **150** | **111*** | **+39** |

\* Measured total reflects camera + rfdetr_node (preprocess/inference/decode); the ByteTrack row is a target budget for the not-yet-implemented tracker and is not included in the current measured total.

### Latency vs. Requirements

| Requirement | Target | Achieved | Status |
|-------------|--------|----------|--------|
| COMP-3: Object Detection | 100ms | 43ms | ✅ 57% margin |
| COMP-4: Object Tracking | 50ms | N/A (not yet implemented) | ⏳ Planned |
| PLAT-1: Sensor → Perception | 150ms | 111ms | ✅ 26% margin |
| E2E-2: Obstacle Avoidance | 250ms | ~200ms | ✅ 20% margin |

---

## Deployment Configuration

### Hardware Requirements

| Component | Specification | Notes |
|-----------|---------------|-------|
| **Compute** | NVIDIA Jetson AGX Orin 64GB | Primary platform |
| **Camera** | Luxonis OAK-D, 1080p @ 30 Hz via USB 3.1 | Stereo depth + IMU |
| **Power** | 30W mode recommended | Balance perf/thermal |

### Jetson Configuration

```bash
# Set power mode (30W for thermal headroom)
sudo nvpmodel -m 2

# Maximize clocks for inference
sudo jetson_clocks

# Verify GPU/DLA availability
tegrastats
```

### TensorRT Engine Generation

Export and engine-build are two separate scripts: `export_rfdetr_onnx.py` runs on a workstation GPU, `build_tensorrt_engine.sh` runs on the Jetson.

```bash
# 1. Export RF-DETR-Small to ONNX (workstation, not Jetson)
pip install rfdetr onnx onnxsim onnxruntime polygraphy onnx-graphsurgeon
python scripts/export_rfdetr_onnx.py --output models/RF-DETR-SMALL.onnx

# 2. Copy the ONNX file to the Jetson, then build the FP16 TensorRT engine
scp models/RF-DETR-SMALL.onnx jetson@orin-dev:/home/dev/models/
ssh jetson@orin-dev "cd /path/to/repo && scripts/build_tensorrt_engine.sh"
```

`export_rfdetr_onnx.py` loads `RFDETRSmall()` (Roboflow `rfdetr` pip package) and exports via the library's own exporter (opset 17, `onnxsim` simplification). `build_tensorrt_engine.sh` runs `trtexec --fp16` on the Jetson and writes the engine next to the ONNX file.

### Engine File Management

| File | Path | Description |
|------|------|--------------|
| `RF-DETR-SMALL.onnx` | `/home/dev/models/` | ONNX export from `export_rfdetr_onnx.py` |
| `RF-DETR-SMALL.engine` | `/home/dev/models/` | TensorRT FP16 engine (default `engine_path` for `rfdetr_node`) |

**Note:** TensorRT engines are hardware-specific. Regenerate when changing Jetson device or JetPack version.

---

## Performance Tuning

### Inference Optimization Checklist

- [ ] Use FP16 precision (default, set by `build_tensorrt_engine.sh --fp16`)
- [ ] Set appropriate TensorRT workspace size (`--memPoolSize=workspace:2048` default)
- [ ] Pin CPU cores for non-GPU nodes
- [ ] Profile `rfdetr_node`'s Python pre/post-processing for further latency headroom

### Confidence Threshold Tuning

| Threshold | Precision | Recall | Use Case |
|-----------|-----------|--------|----------|
| 0.9 | High | Low | Minimize false positives |
| 0.7 | Balanced | Balanced | Higher-precision surveying passes |
| **0.5** | Low-moderate | High | **Node default (`confidence_threshold` param)** |

The node's shipped default is `0.5` (favoring recall); operators can raise it via the `confidence_threshold` launch argument for higher-precision passes.

### Resolution Trade-offs

| Resolution | Notes |
|------------|-------|
| **512×512** | **RF-DETR-Small's native input resolution (selected, fixed by the exported engine)** |

Changing input resolution requires re-exporting the ONNX model and rebuilding the TensorRT engine; the current deployment does not use multi-resolution or tiled (SAHI) inference.

---

## Future Enhancements

| Enhancement | Priority | Description |
|-------------|----------|--------------|
| ByteTrack tracker node | High | Persistent track IDs on `/tracked_objects` (see [Object Tracking](#object-tracking)) |
| IR/thermal fusion | High | Dual-stream encoder with mid-level fusion |
| Multi-camera batching | Medium | Batched inference for gimbal + forward cameras |
| SAHI tiled inference | Medium | Improved small object detection at range |
| Fine-tuned model (Phase 2/3) | High | Domain-specific classes, see [perception_finetuning_plan.md](perception_finetuning_plan.md) |
| Classical vision fallback | Low | DO-178C safety backup path |

---

## References

- [RF-DETR (Roboflow) GitHub](https://github.com/roboflow/rf-detr) - model, training/export code, and current benchmarks
- RT-DETR paper (background/predecessor architecture): [Zhao et al., "DETRs Beat YOLOs on Real-time Object Detection", arXiv:2304.08069](https://arxiv.org/abs/2304.08069) - not the deployed model, cited only for background on real-time DETR-family detectors
- [ByteTrack Paper](https://arxiv.org/abs/2110.06864) - ByteTrack: Multi-Object Tracking by Associating Every Detection Box
- [Latency Requirements](latency_requirements.md) - Platform latency specifications
