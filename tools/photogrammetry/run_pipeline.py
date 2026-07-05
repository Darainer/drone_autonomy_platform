#!/usr/bin/env python3
"""run_pipeline.py — DES-005 CLI entry point (placeholder, T3.1 skeleton only).

Final behavior (T3.2/T3.3 fill this in):
  --mode check  -- validate dataset, predict coverage % from footprint union
                 vs. survey polygon, no reconstruction. T3.2 lands the MAP-7
                 marker here.
  --mode full   -- check stage first (fail fast), then ODM reconstruction,
                 then post-reconstruction coverage QA, then report. T3.3 lands
                 the MAP-4/MAP-8 markers here, T3.2 the MAP-5 marker.

This file only wires up the argparse entry point for T3.1; both modes are
TODO stubs. No traceability markers land in this skeleton -- the real
markers are added by T3.2/T3.3 in the actual implementations.
"""
from __future__ import annotations

import argparse
import sys


def _mode_check(dataset_dir: str, polygon_from_manifest: bool) -> int:
    # TODO(T3.2): dataset validation + footprint.py/coverage.py; this is where
    # the MAP-7 marker will be added.
    raise NotImplementedError("--mode check is implemented in T3.2")


def _mode_full(dataset_dir: str, polygon_from_manifest: bool) -> int:
    # TODO(T3.3): odm_runner.py invocation + post-reconstruction coverage QA;
    # this is where the MAP-4 and MAP-8 markers will be added.
    raise NotImplementedError("--mode full is implemented in T3.3")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="DES-005 post-flight photogrammetry pipeline (check/full modes)."
    )
    parser.add_argument(
        "--mode", choices=["check", "full"], required=True, help="Pipeline mode to run."
    )
    parser.add_argument("--dataset", required=True, help="Path to a DES-004 dataset directory.")
    parser.add_argument(
        "--polygon-from-manifest",
        action="store_true",
        help="Use the survey polygon recorded in manifest.yaml (default source).",
    )
    return parser


def main(argv=None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    if args.mode == "check":
        return _mode_check(args.dataset, args.polygon_from_manifest)
    return _mode_full(args.dataset, args.polygon_from_manifest)


if __name__ == "__main__":
    sys.exit(main())
