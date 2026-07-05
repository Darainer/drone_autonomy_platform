#!/usr/bin/env python3
"""run_pipeline.py — DES-005 CLI entry point.

  --mode check  -- validate dataset, predict coverage % from footprint union
                 vs. survey polygon, no reconstruction (MAP-7, T3.2).
  --mode full   -- check stage first (fail fast), then ODM reconstruction,
                 then post-reconstruction coverage QA, then report (MAP-4/
                 MAP-8, T3.3 -- still a TODO stub in this file).
"""
from __future__ import annotations

import argparse
import pathlib
import sys
from typing import Any, Dict, List

from photogrammetry import coverage, dataset as ds, footprint
from verify_dataset import verify_dataset

DEFAULT_MIN_COVERAGE = coverage.DEFAULT_COVERAGE_THRESHOLD
SYNC_ERR_WARN_MS = 50.0


def _sync_error_stats(poses) -> Dict[str, Any]:
    """Mean/max sync_err_ms across all poses, and how many exceed the 50 ms budget.

    DES-004 D4 fixes the ApproximateTime sync slop budget at 50 ms; MAP-7
    requires "frame/pose synchronization health" in the onboard report.
    """
    values = [p.sync_err_ms for p in poses]
    return {
        "frame_count": len(values),
        "mean_ms": (sum(values) / len(values)) if values else None,
        "max_ms": max(values) if values else None,
        "count_over_50ms": sum(1 for v in values if v > SYNC_ERR_WARN_MS),
    }


def _report_dir(dataset_dir: pathlib.Path) -> pathlib.Path:
    return dataset_dir / "products" / "report"


# Implements: MAP-7
def _mode_check(dataset_dir: str, polygon_from_manifest: bool, min_coverage: float) -> int:
    """DES-005 D3 / MAP-7: onboard post-landing consistency check.

    Sequence (no reconstruction -- pure numpy/shapely geometry only, so this
    stays within the 15-minute Orin (arm64) budget for a 1000-frame dataset,
    TP-002 TS-10):
      (a) validity: verify_dataset.verify_dataset() (manifest/checksum/poses);
      (b) sync-error stats from poses.csv sync_err_ms;
      (c) predicted coverage: footprint union (poses x camera model) vs.
          the manifest survey polygon (footprint.py + coverage.py);
      (d) write report/coverage.json + report/coverage.md;
      (e) exit 0 iff dataset valid AND predicted coverage >= min_coverage.

    `polygon_from_manifest` is accepted for CLI symmetry with `full` mode;
    the manifest polygon is DES-005's only defined check-mode polygon source
    in v1 (D3), so it is a no-op today.
    """
    del polygon_from_manifest  # only source implemented in v1 (DES-005 D3)
    dataset_path = pathlib.Path(dataset_dir)

    # (a) validity -- this is the ONLY dataset-integrity check; no
    # reconstruction, no image decoding beyond what verify_dataset already
    # does (sha256 + existence checks), per the check-mode invariant.
    errors: List[str] = verify_dataset(dataset_path)
    if errors:
        print(f"INVALID dataset: {dataset_path}", file=sys.stderr)
        for err in errors:
            print(f"  - {err}", file=sys.stderr)
        try:
            coverage.write_report(
                _report_dir(dataset_path),
                {"coverage_pct": None, "threshold": min_coverage, "verdict": "fail"},
                extra={"valid": False, "mode": "check", "errors": errors},
            )
        except OSError as exc:
            print(f"warning: could not write report: {exc}", file=sys.stderr)
        return 1

    manifest = ds.load_manifest(dataset_path)
    poses = ds.read_poses(dataset_path)

    # (b) sync-error stats.
    sync_stats = _sync_error_stats(poses)

    # (c) predicted coverage: per-frame ground footprint (camera model x
    # pose) unioned, then intersected with the manifest survey polygon.
    # No image pixels are read here -- geometry only (numpy + shapely).
    try:
        footprints = footprint.footprints_for_poses(manifest["camera_intrinsics"], poses)
    except footprint.FootprintError as exc:
        print(f"INVALID dataset: {dataset_path}", file=sys.stderr)
        print(f"  - {exc}", file=sys.stderr)
        try:
            coverage.write_report(
                _report_dir(dataset_path),
                {"coverage_pct": None, "threshold": min_coverage, "verdict": "fail"},
                extra={
                    "valid": False,
                    "mode": "check",
                    "errors": [str(exc)],
                    "sync_error_stats": sync_stats,
                },
            )
        except OSError as write_exc:
            print(f"warning: could not write report: {write_exc}", file=sys.stderr)
        return 1

    predicted_union = footprint.union_footprints(footprints)
    survey_polygon = coverage.survey_polygon_from_manifest(manifest)
    coverage_result = coverage.compute_coverage(predicted_union, survey_polygon, threshold=min_coverage)

    # (d) write report.
    coverage.write_report(
        _report_dir(dataset_path),
        coverage_result,
        extra={
            "valid": True,
            "mode": "check",
            "sync_error_stats": sync_stats,
        },
    )

    print(
        f"check mode: predicted coverage {coverage_result['coverage_pct']:.2f}% "
        f"(threshold {min_coverage:.2f}%) -- verdict {coverage_result['verdict']}"
    )

    # (e) exit 0 iff dataset valid (already confirmed above) AND predicted
    # coverage >= min_coverage.
    return 0 if coverage_result["verdict"] == "pass" else 1


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
    parser.add_argument(
        "--min-coverage",
        type=float,
        default=DEFAULT_MIN_COVERAGE,
        help=f"Predicted/actual coverage %% gate threshold (default {DEFAULT_MIN_COVERAGE}).",
    )
    return parser


def main(argv=None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    if args.mode == "check":
        return _mode_check(args.dataset, args.polygon_from_manifest, args.min_coverage)
    return _mode_full(args.dataset, args.polygon_from_manifest)


if __name__ == "__main__":
    sys.exit(main())
