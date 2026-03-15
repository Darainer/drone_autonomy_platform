#!/bin/bash
# Build TensorRT engine from the RT-DETR-R50 COCO ONNX model.
# Must be run on the target GPU (Jetson Orin). Takes ~10-15 minutes.
#
# trtexec is provided by the libnvinfer-bin package, pre-installed in the
# NVIDIA Isaac ROS base image (nvcr.io/nvidia/isaac/ros).
set -e

MODELS_DIR="${MODELS_DIR:-/home/dev/models}"
ONNX_FILE="${MODELS_DIR}/rtdetr_coco.onnx"
ENGINE_FILE="${MODELS_DIR}/rtdetr_coco.plan"
TRTEXEC="${TENSORRT_COMMAND:-/usr/src/tensorrt/bin/trtexec}"

if [ ! -f "${ONNX_FILE}" ]; then
    echo "ERROR: ONNX model not found at ${ONNX_FILE}"
    echo "Re-build the Docker image or run manually:"
    echo "  scripts/export_rtdetr_onnx.sh"
    exit 1
fi

if [ -f "${ENGINE_FILE}" ]; then
    echo "Engine already exists at ${ENGINE_FILE}"
    echo "Delete it first if you want to rebuild."
    exit 0
fi

echo "Building TensorRT engine from ${ONNX_FILE}..."
echo "This will take ~10-15 minutes on Jetson Orin Nano (faster on AGX Orin)."

"${TRTEXEC}" \
    --onnx="${ONNX_FILE}" \
    --saveEngine="${ENGINE_FILE}" \
    --fp16

echo "Done: ${ENGINE_FILE}"
