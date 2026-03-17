#!/bin/bash
# Build TensorRT FP16 engine from RF-DETR-S ONNX model.
# Must be run on the target GPU (Jetson Orin Nano). Takes ~10-20 minutes.
#
# Prerequisites:
#   - ONNX model exported via: python scripts/export_rfdetr_onnx.py
#   - trtexec from libnvinfer-bin (pre-installed in Isaac ROS base image)
#
# RF-DETR-S: 32.1M params, 512x512 input, NMS-free (Apache 2.0)
set -e

MODELS_DIR="${MODELS_DIR:-/home/dev/models}"
ONNX_FILE="${1:-${MODELS_DIR}/rfdetr_s_coco.onnx}"
ENGINE_FILE="${ONNX_FILE%.onnx}.engine"
TRTEXEC="${TENSORRT_COMMAND:-/usr/src/tensorrt/bin/trtexec}"

if [ ! -f "${ONNX_FILE}" ]; then
    echo "ERROR: ONNX model not found at ${ONNX_FILE}"
    echo "Export it first on a workstation:"
    echo "  python scripts/export_rfdetr_onnx.py --output ${ONNX_FILE}"
    exit 1
fi

if [ -f "${ENGINE_FILE}" ]; then
    echo "Engine already exists at ${ENGINE_FILE}"
    echo "Delete it first if you want to rebuild."
    exit 0
fi

echo "============================================"
echo "  RF-DETR-S TensorRT Engine Build"
echo "============================================"
echo "  ONNX:    ${ONNX_FILE}"
echo "  Engine:  ${ENGINE_FILE}"
echo "  Mode:    FP16"
echo "  Input:   1x3x512x512 (static)"
echo "============================================"
echo ""
echo "This will take ~10-20 minutes on Jetson Orin Nano."
echo ""

"${TRTEXEC}" \
    --onnx="${ONNX_FILE}" \
    --saveEngine="${ENGINE_FILE}" \
    --fp16 \
    --workspace=2048 \
    --verbose 2>&1 | tee "${ENGINE_FILE%.engine}_build.log"

echo ""
echo "Engine built: ${ENGINE_FILE}"
echo "Build log:    ${ENGINE_FILE%.engine}_build.log"

# Quick validation
echo ""
echo "Running latency benchmark (10 iterations)..."
"${TRTEXEC}" \
    --loadEngine="${ENGINE_FILE}" \
    --iterations=10 \
    --warmUp=1000 \
    --avgRuns=10 2>&1 | grep -E "(mean|median|percentile|GPU Compute Time)"

echo ""
echo "Done. Copy this engine to the Jetson and set model_path in rfdetr.launch.py."
