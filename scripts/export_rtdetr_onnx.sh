#!/bin/bash
# Export RT-DETR-R50 COCO ONNX from the official lyuwenyu/RT-DETR repo (Apache 2.0).
# Runs at Docker build time. Skip if ONNX already exists (e.g. bind-mounted from host).
#
# Weights: rtdetr_r50vd_6x_coco_from_paddle.pth — 53.0 mAP COCO val2017
set -e

MODELS_DIR="${MODELS_DIR:-/home/dev/models}"
ONNX_OUT="${MODELS_DIR}/rtdetr_coco.onnx"
REPO_DIR="/tmp/rtdetr_export"
WEIGHTS_URL="https://github.com/lyuwenyu/RT-DETR/releases/download/v0.1/rtdetr_r50vd_6x_coco_from_paddle.pth"

if [ -f "${ONNX_OUT}" ]; then
    echo "RT-DETR ONNX already exists at ${ONNX_OUT}, skipping export."
    exit 0
fi

mkdir -p "${MODELS_DIR}"

echo "Cloning lyuwenyu/RT-DETR..."
git clone --depth=1 https://github.com/lyuwenyu/RT-DETR.git "${REPO_DIR}"

echo "Installing requirements..."
pip3 install -q -r "${REPO_DIR}/rtdetr_pytorch/requirements.txt"

echo "Downloading RT-DETR-R50 COCO weights..."
wget -q -O "${REPO_DIR}/rtdetr_pytorch/rtdetr_r50vd_6x_coco_from_paddle.pth" "${WEIGHTS_URL}"

echo "Exporting ONNX..."
cd "${REPO_DIR}/rtdetr_pytorch"
python3 tools/export_onnx.py \
    -c configs/rtdetr/rtdetr_r50vd_6x_coco.yml \
    -r rtdetr_r50vd_6x_coco_from_paddle.pth \
    --check

# export_onnx.py writes to rtdetr_r50vd_6x_coco.onnx in cwd
cp rtdetr_r50vd_6x_coco.onnx "${ONNX_OUT}"

echo "Cleaning up..."
rm -rf "${REPO_DIR}"

echo "Done: ${ONNX_OUT}"
