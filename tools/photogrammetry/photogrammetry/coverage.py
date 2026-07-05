"""Polygon coverage percentage computation + report writer.

# Implements: MAP-5

Engine-agnostic per DES-005 D4: given a "covered region" shapely geometry
(the predicted footprint union in `--mode check`, or the reconstruction
footprint in `--mode full` -- T3.3) and the survey polygon, coverage_pct is
the shapely intersection-over-polygon area ratio. This module does not
care how the covered region was produced -- it is used by both modes.
"""
from __future__ import annotations

import json
import pathlib
from typing import Any, Dict, List, Optional

from shapely.geometry import Polygon, base as shapely_base

DEFAULT_COVERAGE_THRESHOLD = 95.0

COVERAGE_JSON_FILENAME = "coverage.json"
COVERAGE_MD_FILENAME = "coverage.md"


def survey_polygon_from_manifest(manifest: Dict[str, Any]) -> Polygon:
    """Build the shapely survey polygon from a DES-004 manifest's `polygon` key."""
    vertices = [(float(v[0]), float(v[1])) for v in manifest["polygon"]]
    return Polygon(vertices)


# Implements: MAP-5
def compute_coverage(
    covered_region: shapely_base.BaseGeometry,
    survey_polygon: shapely_base.BaseGeometry,
    threshold: float = DEFAULT_COVERAGE_THRESHOLD,
) -> Dict[str, Any]:
    """Compute coverage_pct = area(covered ∩ survey_polygon) / area(survey_polygon) * 100.

    Per DES-005 D4 / TS-09: a shapely intersection-over-polygon area ratio.
    Returns a dict with `coverage_pct`, `threshold`, and `verdict` ("pass"
    iff coverage_pct >= threshold, else "fail").

    Raises ValueError if the survey polygon has zero (or degenerate) area,
    since coverage is undefined in that case.
    """
    survey_area = survey_polygon.area
    if survey_area <= 0.0:
        raise ValueError(f"survey polygon has non-positive area ({survey_area}); cannot compute coverage")

    if covered_region.is_empty:
        intersection_area = 0.0
    else:
        intersection_area = covered_region.intersection(survey_polygon).area

    coverage_pct = (intersection_area / survey_area) * 100.0
    verdict = "pass" if coverage_pct >= threshold else "fail"
    return {
        "coverage_pct": coverage_pct,
        "threshold": threshold,
        "verdict": verdict,
    }


def write_report(
    report_dir: pathlib.Path,
    coverage_result: Dict[str, Any],
    extra: Optional[Dict[str, Any]] = None,
) -> Dict[str, pathlib.Path]:
    """Write `coverage.json` (machine-readable) and `coverage.md` (human) under `report_dir`.

    `coverage_result` is expected to look like the dict returned by
    `compute_coverage` (numeric `coverage_pct` may be None for a dataset
    that failed validity checks before coverage could be computed).
    `extra` is merged into the JSON report verbatim (e.g. sync-error stats,
    frame count, validity errors) and rendered into the markdown report on a
    best-effort basis.
    """
    report_dir = pathlib.Path(report_dir)
    report_dir.mkdir(parents=True, exist_ok=True)

    payload: Dict[str, Any] = dict(coverage_result)
    if extra:
        payload.update(extra)

    json_path = report_dir / COVERAGE_JSON_FILENAME
    with open(json_path, "w") as f:
        json.dump(payload, f, indent=2, sort_keys=True)
        f.write("\n")

    md_path = report_dir / COVERAGE_MD_FILENAME
    with open(md_path, "w") as f:
        f.write(_render_markdown(payload))

    return {"json": json_path, "md": md_path}


def _render_markdown(payload: Dict[str, Any]) -> str:
    lines: List[str] = ["# Coverage Report", ""]

    coverage_pct = payload.get("coverage_pct")
    threshold = payload.get("threshold")
    verdict = payload.get("verdict", "unknown")

    coverage_str = f"{coverage_pct:.2f}%" if isinstance(coverage_pct, (int, float)) else "n/a"
    threshold_str = f"{threshold:.2f}%" if isinstance(threshold, (int, float)) else "n/a"

    lines.append(f"- **Coverage:** {coverage_str}")
    lines.append(f"- **Threshold:** {threshold_str}")
    lines.append(f"- **Verdict:** `{verdict}`")

    sync_stats = payload.get("sync_error_stats")
    if isinstance(sync_stats, dict):
        lines.append("")
        lines.append("## Sync-error stats")
        lines.append(f"- mean_ms: {sync_stats.get('mean_ms')}")
        lines.append(f"- max_ms: {sync_stats.get('max_ms')}")
        lines.append(f"- count_over_50ms: {sync_stats.get('count_over_50ms')}")
        lines.append(f"- frame_count: {sync_stats.get('frame_count')}")

    errors = payload.get("errors")
    if errors:
        lines.append("")
        lines.append("## Validity errors")
        for err in errors:
            lines.append(f"- {err}")

    remaining_keys = {
        k: v
        for k, v in payload.items()
        if k not in {"coverage_pct", "threshold", "verdict", "sync_error_stats", "errors"}
    }
    if remaining_keys:
        lines.append("")
        lines.append("## Other")
        for k, v in sorted(remaining_keys.items()):
            lines.append(f"- {k}: {v}")

    lines.append("")
    return "\n".join(lines)
