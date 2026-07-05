#!/usr/bin/env python3
"""run_pipeline.py — DES-005 CLI entry point.

  --mode check  -- validate dataset, predict coverage % from footprint union
                 vs. survey polygon, no reconstruction (MAP-7, T3.2).
  --mode full   -- check stage first (fail fast), then ODM reconstruction,
                 then post-reconstruction coverage QA, then report (MAP-4/
                 MAP-8, T3.3).
"""
from __future__ import annotations

import argparse
import pathlib
import platform
import sys
from typing import Any, Dict, List, Optional

from photogrammetry import coverage, dataset as ds, footprint, odm_runner
from verify_dataset import collect_warnings, verify_dataset

DEFAULT_MIN_COVERAGE = coverage.DEFAULT_COVERAGE_THRESHOLD
SYNC_ERR_WARN_MS = 50.0

# Recognized execution targets (DES-005: "the reference ground station,
# x86_64" vs. "the Orin Nano class companion, arm64"). `platform.machine()`
# spells these various ways across platforms/kernels; normalize.
ARCH_GROUND_STATION = "x86_64"
ARCH_ONBOARD = "aarch64"
_ARCH_ALIASES = {
    "x86_64": ARCH_GROUND_STATION,
    "amd64": ARCH_GROUND_STATION,
    "aarch64": ARCH_ONBOARD,
    "arm64": ARCH_ONBOARD,
}


# Implements: MAP-8
def detect_execution_target(machine: Optional[str] = None) -> str:
    """Identify the execution target (ground-station x86_64 vs. onboard arm64/Jetson).

    MAP-8 requires the same package/CLI to run `--mode full` on both the
    x86_64 ground station and the aarch64 Jetson/Orin companion (the ODM
    image is itself multi-arch, so no branching on the ODM invocation
    itself is needed -- this only identifies and logs which target is
    running, for the report/operator). Returns a normalized arch string
    (`ARCH_GROUND_STATION` / `ARCH_ONBOARD`), or the raw `platform.machine()`
    value verbatim if it is an arch this function does not recognize
    (never raises -- an unrecognized arch just runs the identical pipeline
    with an unrecognized-but-logged label).
    """
    raw = machine if machine is not None else platform.machine()
    return _ARCH_ALIASES.get(raw, raw)


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

    # F-6(b): non-fatal image-dims-vs-manifest-intrinsics consistency
    # warning. A single Pillow header-only open (no pixel decode) -- cheap
    # enough to keep the check-mode no-reconstruction invariant. Never
    # affects the verdict/exit code below.
    warnings: List[str] = collect_warnings(dataset_path)
    for warning in warnings:
        print(f"WARNING: {warning}", file=sys.stderr)

    if errors:
        print(f"INVALID dataset: {dataset_path}", file=sys.stderr)
        for err in errors:
            print(f"  - {err}", file=sys.stderr)
        try:
            coverage.write_report(
                _report_dir(dataset_path),
                {"coverage_pct": None, "threshold": min_coverage, "verdict": "fail"},
                extra={"valid": False, "mode": "check", "errors": errors, "warnings": warnings},
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
                    "warnings": warnings,
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
            "warnings": warnings,
        },
    )

    print(
        f"check mode: predicted coverage {coverage_result['coverage_pct']:.2f}% "
        f"(threshold {min_coverage:.2f}%) -- verdict {coverage_result['verdict']}"
    )

    # (e) exit 0 iff dataset valid (already confirmed above) AND predicted
    # coverage >= min_coverage.
    return 0 if coverage_result["verdict"] == "pass" else 1


# Implements: MAP-4
def _mode_full(
    dataset_dir: str,
    polygon_from_manifest: bool,
    min_coverage: float,
    odm_runner_fn=None,
) -> int:
    """DES-005 D1/D4/D6 / MAP-4 (ground station) / MAP-8 (onboard): full reconstruction.

    Sequence (STK-1(d): fully unattended, stdin closed, zero prompts):
      (a) check stage FIRST (fail fast) -- dataset validity via
          verify_dataset(); an invalid dataset never reaches ODM;
      (b) invoke odm_runner (unattended container run) -> collect D6
          products (.laz/.obj+.mtl/.tif);
      (c) post-reconstruction coverage QA (D4): reconstruction footprint vs.
          the manifest survey polygon, via coverage.compute_coverage();
      (d) write products/report/coverage.json + coverage.md;
      (e) exit 0 iff products complete AND coverage_pct >= min_coverage.

    `polygon_from_manifest` is accepted for CLI symmetry with `check` mode
    (the manifest polygon is DES-005's only defined survey-polygon source in
    v1, same as check mode -- see `_mode_check`'s docstring).

    `odm_runner_fn` is the injectable test seam: `(dataset_dir, manifest,
    poses) -> odm_runner.OdmRunResult`-shaped callable (defaults to
    `odm_runner.run_odm`). This lets the TS-08 smoke test substitute a fake
    ODM runner and drive this function end-to-end without Docker.
    """
    del polygon_from_manifest  # only source implemented in v1 (DES-005 D3/D4 symmetry)
    dataset_path = pathlib.Path(dataset_dir)

    target = detect_execution_target()
    print(f"full mode: execution target detected as {target} ({platform.machine()})")

    # (a) validity -- fail fast, never invoke ODM on an invalid dataset.
    errors: List[str] = verify_dataset(dataset_path)
    if errors:
        print(f"INVALID dataset: {dataset_path}", file=sys.stderr)
        for err in errors:
            print(f"  - {err}", file=sys.stderr)
        try:
            coverage.write_report(
                _report_dir(dataset_path),
                {"coverage_pct": None, "threshold": min_coverage, "verdict": "fail"},
                extra={"valid": False, "mode": "full", "errors": errors, "execution_target": target},
            )
        except OSError as exc:
            print(f"warning: could not write report: {exc}", file=sys.stderr)
        return 1

    manifest = ds.load_manifest(dataset_path)
    poses = ds.read_poses(dataset_path)

    # (b) ODM reconstruction (unattended -- STK-1(d): stdin closed, no
    # -i/-t/-it, no interactive prompts on either side of this call).
    run_odm = odm_runner_fn if odm_runner_fn is not None else odm_runner.run_odm
    try:
        run_odm(dataset_path, manifest, poses)
    except odm_runner.OdmRunnerError as exc:
        print(f"ODM reconstruction failed: {exc}", file=sys.stderr)
        try:
            coverage.write_report(
                _report_dir(dataset_path),
                {"coverage_pct": None, "threshold": min_coverage, "verdict": "fail"},
                extra={
                    "valid": True,
                    "mode": "full",
                    "errors": [str(exc)],
                    "execution_target": target,
                },
            )
        except OSError as write_exc:
            print(f"warning: could not write report: {write_exc}", file=sys.stderr)
        return 1

    products_ok = odm_runner.products_complete(dataset_path)

    # (c) post-reconstruction coverage QA (D4): reconstruction footprint vs.
    # the manifest survey polygon, BOTH in the local ENU frame (DES-004 D3 --
    # ODM georeferences via GNSS, but QA is done in local ENU; the poses carry
    # both lon/lat and ENU x/y so the footprint is reconciled into ENU).
    try:
        recon_footprint = odm_runner.reconstruction_footprint(dataset_path, poses)
        survey_polygon = coverage.survey_polygon_from_manifest(manifest)
        coverage_result = coverage.compute_coverage(recon_footprint, survey_polygon, threshold=min_coverage)
    except (odm_runner.OdmRunnerError, ValueError) as exc:
        print(f"coverage QA failed: {exc}", file=sys.stderr)
        try:
            coverage.write_report(
                _report_dir(dataset_path),
                {"coverage_pct": None, "threshold": min_coverage, "verdict": "fail"},
                extra={
                    "valid": True,
                    "mode": "full",
                    "errors": [str(exc)],
                    "products_complete": products_ok,
                    "execution_target": target,
                },
            )
        except OSError as write_exc:
            print(f"warning: could not write report: {write_exc}", file=sys.stderr)
        return 1

    # (d) write report.
    coverage.write_report(
        _report_dir(dataset_path),
        coverage_result,
        extra={
            "valid": True,
            "mode": "full",
            "products_complete": products_ok,
            "execution_target": target,
        },
    )

    print(
        f"full mode: reconstruction coverage {coverage_result['coverage_pct']:.2f}% "
        f"(threshold {min_coverage:.2f}%) -- verdict {coverage_result['verdict']}, "
        f"products_complete={products_ok}"
    )

    # (e) exit 0 iff products complete AND coverage_pct >= min_coverage.
    return 0 if (products_ok and coverage_result["verdict"] == "pass") else 1


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
    return _mode_full(args.dataset, args.polygon_from_manifest, args.min_coverage)


if __name__ == "__main__":
    sys.exit(main())
