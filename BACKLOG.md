# Perception Pipeline Backlog

Issues found during first launch attempt on Orin.

## Active

### ~~1. VSLAM: missing libgxf_serialization.so~~ DONE
- Fixed: added GXF lib paths to `LD_LIBRARY_PATH` in Dockerfile.orin

### 2. RT-DETR: model file not found + OOM
- Default `model_path` points to `/workspaces/isaac_ros-dev/models/rtdetr_l.plan` (doesn't exist)
- After failing to load model, TensorRT tries default 64MB block allocation and hits `GXF_OUT_OF_MEMORY`
- Fix: provide correct model path, generate TensorRT engine from ONNX

### 3. OAK-D: USB device permissions
- `Insufficient permissions to communicate with X_LINK_UNBOOTED device`
- Container has `privileged: true` but udev rules may be missing on host
- Fix: install udev rules on host (`depthai-python` provides them) or pass device explicitly

## Done

- [x] Fix VSLAM executable name (`visual_slam_node` → `isaac_ros_visual_slam`)
- [x] Add colcon build step to Dockerfile.orin
- [x] Add colcon build step to Dockerfile.dev
