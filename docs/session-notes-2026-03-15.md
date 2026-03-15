# Session Notes — 2026-03-15

### Model swap — sdetr_grasp → RT-DETR-R50 COCO
- `sdetr_grasp` is a robotic grasping model, completely wrong for drone use
- RT-DETR-R50 COCO chosen: Apache 2.0, 53.0 mAP, tensor names match Isaac ROS exactly
- COCO covers: person, bird, car, truck, bus, horse, cow, sheep, bear + more
- Still need fine-tuning for: deer, tractors/farm equipment
- `scripts/export_rtdetr_onnx.sh` — exports ONNX from lyuwenyu/RT-DETR at Docker build time
- `scripts/build_tensorrt_engine.sh` — updated to use `rtdetr_coco.onnx` / `rtdetr_coco.plan`
- **Next step: rebuild Docker image** to get the COCO ONNX, then build TRT engine on first run

## What still needs doing
- [ ] Rebuild `drone_autonomy_platform:orin` image to get `rtdetr_coco.onnx`
- [ ] Build `rtdetr_coco.plan` TRT engine on first container run (~10-15 min)
- [ ] Verify detections flowing with COCO model (people/vehicles should detect)
- [ ] VSLAM — not working: needs FC connected for MAVROS IMU (`/mavros/imu/data`)
- [ ] IMU firmware update for OAK-D (3.2.13 → 3.9.9) — OAK-D may not have IMU hardware
- [ ] Fine-tune model

## Colcon build command (always needed after launch file changes)
```bash
rm -rf /ros2_ws/build/perception && cd /ros2_ws && colcon build \
  --packages-select perception --merge-install \
  --base-paths /ros2_ws/src/drone_autonomy_platform/msgs /ros2_ws/src/drone_autonomy_platform/src \
  && source install/setup.bash
```
