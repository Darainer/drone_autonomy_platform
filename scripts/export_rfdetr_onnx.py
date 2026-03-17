#!/usr/bin/env python3
"""Export RF-DETR-S to ONNX for TensorRT deployment on Jetson Orin Nano.

Run this on a workstation with a GPU (not on the Jetson itself).
The resulting ONNX file is then converted to a TensorRT engine on-device.

Usage:
    pip install rfdetr onnx onnxsim
    python scripts/export_rfdetr_onnx.py [--output /path/to/rfdetr_s.onnx]

RF-DETR-S specs (COCO val2017):
    - 53.0 mAP @ 512x512, 32.1M params, ~71 GFLOPs
    - NMS-free (end-to-end), Apache 2.0 license
"""

import argparse
import sys
from pathlib import Path

import onnx
import torch


def export_rfdetr_s(output_path: str, opset: int = 17, simplify: bool = True) -> Path:
    """Export RF-DETR-S checkpoint to ONNX with static input shape."""
    from rfdetr import RFDETRSmall

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)

    print("Loading RF-DETR-S pretrained weights (COCO)...")
    model = RFDETRSmall(pretrained=True)
    model.model.eval()

    # RF-DETR-S uses 512x512 input resolution
    input_h, input_w = 512, 512
    dummy_input = torch.randn(1, 3, input_h, input_w)

    print(f"Exporting to ONNX (opset {opset}, input: 1x3x{input_h}x{input_w})...")
    torch.onnx.export(
        model.model,
        dummy_input,
        str(output),
        opset_version=opset,
        input_names=["images"],
        output_names=["labels", "boxes", "scores"],
        dynamic_axes=None,  # static shape for TensorRT optimization
    )

    if simplify:
        print("Simplifying ONNX graph...")
        try:
            import onnxsim

            onnx_model = onnx.load(str(output))
            onnx_model, check = onnxsim.simplify(onnx_model)
            if check:
                onnx.save(onnx_model, str(output))
                print("ONNX simplification succeeded.")
            else:
                print("WARNING: ONNX simplification check failed, keeping original.")
        except ImportError:
            print("WARNING: onnxsim not installed, skipping simplification.")

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
        print("Install with: pip install rfdetr onnx onnxsim", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
