#!/bin/bash
# Build a TensorRT FP16 engine from an ONNX model on the target GPU.
# Defaults to the exported RF-DETR-S model but also supports the legacy
# SyntheticaDETR path by passing an explicit ONNX file.
set -euo pipefail

MODELS_DIR="${MODELS_DIR:-/home/dev/models}"
DEFAULT_ONNX_FILE="${MODELS_DIR}/RF-DETR-SMALL.onnx"
LEGACY_ONNX_FILE="${MODELS_DIR}/sdetr_grasp.onnx"
ONNX_FILE="${1:-${DEFAULT_ONNX_FILE}}"
TRTEXEC="${TENSORRT_COMMAND:-/usr/src/tensorrt/bin/trtexec}"

if [ ! -f "${ONNX_FILE}" ] && [ "${ONNX_FILE}" = "${DEFAULT_ONNX_FILE}" ] && [ -f "${LEGACY_ONNX_FILE}" ]; then
    ONNX_FILE="${LEGACY_ONNX_FILE}"
fi

if [ "${ONNX_FILE}" = "${LEGACY_ONNX_FILE}" ]; then
    ENGINE_FILE="${MODELS_DIR}/sdetr_grasp.plan"
else
    ENGINE_FILE="${ONNX_FILE%.onnx}.engine"
fi
BUILD_LOG="${ENGINE_FILE%.*}_build.log"

if [ ! -f "${ONNX_FILE}" ]; then
    echo "ERROR: ONNX model not found at ${ONNX_FILE}"
    echo "Usage: $0 [path/to/model.onnx]"
    echo "Example: $0 ${MODELS_DIR}/RF-DETR-SMALL.onnx"
    exit 1
fi

if [ -f "${ENGINE_FILE}" ]; then
    echo "Engine already exists at ${ENGINE_FILE}"
    echo "Delete it first if you want to rebuild."
    exit 0
fi

if [ ! -x "${TRTEXEC}" ]; then
    echo "ERROR: trtexec not found at ${TRTEXEC}"
    exit 1
fi

echo "============================================"
echo "  TensorRT Engine Build"
echo "============================================"
echo "  ONNX:    ${ONNX_FILE}"
echo "  Engine:  ${ENGINE_FILE}"
echo "  Mode:    FP16"
echo "============================================"
echo ""
echo "This will take several minutes on Jetson Orin."
echo ""

"${TRTEXEC}" \
    --onnx="${ONNX_FILE}" \
    --saveEngine="${ENGINE_FILE}" \
    --fp16 \
    --memPoolSize=workspace:2048 \
    --verbose 2>&1 | tee "${BUILD_LOG}"

echo ""
echo "Engine built: ${ENGINE_FILE}"
echo "Build log:    ${BUILD_LOG}"

echo ""
echo "Running latency benchmark (10 iterations)..."
"${TRTEXEC}" \
    --loadEngine="${ENGINE_FILE}" \
    --iterations=10 \
    --warmUp=1000 \
    --avgRuns=10 2>&1 | grep -E "(mean|median|percentile|GPU Compute Time)" || true
