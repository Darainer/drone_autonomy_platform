#!/bin/bash
# Build TensorRT engine from the SyntheticaDETR ONNX model.
# Must be run on the target GPU (Jetson Orin). Takes ~10-15 minutes.
set -e

MODELS_DIR="${MODELS_DIR:-/home/dev/models}"
ONNX_FILE="${MODELS_DIR}/sdetr_grasp.onnx"
ENGINE_FILE="${MODELS_DIR}/sdetr_grasp.plan"
TRTEXEC="${TENSORRT_COMMAND:-/usr/src/tensorrt/bin/trtexec}"

if [ ! -f "${ONNX_FILE}" ]; then
    echo "ERROR: ONNX model not found at ${ONNX_FILE}"
    echo "Re-build the Docker image or download manually:"
    echo "  wget -O ${ONNX_FILE} https://api.ngc.nvidia.com/v2/models/nvidia/isaac/synthetica_detr/versions/1.0.0_onnx/files/sdetr_grasp.onnx"
    exit 1
fi

if [ -f "${ENGINE_FILE}" ]; then
    echo "Engine already exists at ${ENGINE_FILE}"
    echo "Delete it first if you want to rebuild."
    exit 0
fi

echo "Building TensorRT engine from ${ONNX_FILE}..."
echo "This will take ~10-15 minutes on Jetson AGX Orin."

"${TRTEXEC}" \
    --onnx="${ONNX_FILE}" \
    --saveEngine="${ENGINE_FILE}" \
    --fp16

echo "Done: ${ENGINE_FILE}"
