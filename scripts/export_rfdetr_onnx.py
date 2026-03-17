#!/usr/bin/env python3
"""Export RF-DETR-S to ONNX for TensorRT deployment on Jetson Orin Nano.

Run this on a workstation with a GPU (not on the Jetson itself).
The resulting ONNX file is then converted to a TensorRT engine on-device.

Usage:
    pip install rfdetr onnx onnxsim onnxruntime polygraphy onnx-graphsurgeon
    python scripts/export_rfdetr_onnx.py [--output /path/to/rfdetr_s.onnx]

RF-DETR-S specs (COCO val2017):
    - 53.0 mAP @ 512x512, 32.1M params, ~71 GFLOPs
    - NMS-free (end-to-end), Apache 2.0 license
"""

import argparse
import shutil
import sys
from pathlib import Path

import onnx


def export_rfdetr_s(output_path: str, opset: int = 17, simplify: bool = True) -> Path:
    """Export RF-DETR-S checkpoint to ONNX with static input shape."""
    from rfdetr import RFDETRSmall

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)

    print("Loading RF-DETR-S pretrained weights (COCO)...")
    model = RFDETRSmall()
    export_dir = output.parent
    print(f"Exporting to ONNX via RF-DETR library exporter (opset {opset})...")
    model.export(
        output_dir=str(export_dir),
        simplify=simplify,
        opset_version=opset,
        verbose=False,
        batch_size=1,
    )

    exported_name = "inference_model.sim.onnx" if simplify else "inference_model.onnx"
    exported_path = export_dir / exported_name
    if not exported_path.exists():
        raise FileNotFoundError(f"Expected exported ONNX file not found: {exported_path}")
    if exported_path != output:
        shutil.move(str(exported_path), str(output))

    # Validate
    onnx_model = onnx.load(str(output))
    onnx.checker.check_model(onnx_model)

    size_mb = output.stat().st_size / (1024 * 1024)
    print(f"Exported: {output} ({size_mb:.1f} MB)")
    print(f"Inputs:  {[i.name for i in onnx_model.graph.input]}")
    print(f"Outputs: {[o.name for o in onnx_model.graph.output]}")
    return output


def main():
    parser = argparse.ArgumentParser(description="Export RF-DETR-S to ONNX")
    parser.add_argument(
        "--output",
        default="models/rfdetr_s_coco.onnx",
        help="Output ONNX file path (default: models/rfdetr_s_coco.onnx)",
    )
    parser.add_argument("--opset", type=int, default=17, help="ONNX opset version")
    parser.add_argument(
        "--no-simplify", action="store_true", help="Skip onnxsim simplification"
    )
    args = parser.parse_args()

    try:
        export_rfdetr_s(args.output, args.opset, simplify=not args.no_simplify)
    except ImportError as e:
        print(f"ERROR: Missing dependency: {e}", file=sys.stderr)
        print(
            "Install with: pip install rfdetr onnx onnxsim onnxruntime polygraphy onnx-graphsurgeon",
            file=sys.stderr,
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
