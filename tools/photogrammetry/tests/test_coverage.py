"""Tests for photogrammetry.coverage — coverage %% computation + report writer.

# TS-09 -- Verifies: MAP-5
"""
from __future__ import annotations

import json

from shapely.geometry import Polygon

from photogrammetry import coverage as cov

# REF-RECT (TP-002 F1): 100 m x 80 m rectangle, area = 8000 sq m.
REF_RECT = Polygon([(0.0, 0.0), (100.0, 0.0), (100.0, 80.0), (0.0, 80.0)])


def _centered_hole(area_fraction: float) -> Polygon:
    """A rectangular hole centered in REF-RECT whose area is `area_fraction` of it."""
    rect_area = REF_RECT.area
    hole_area = rect_area * area_fraction
    # Keep it a simple rectangle: fix hole height = 20m, derive width.
    hole_h = 20.0
    hole_w = hole_area / hole_h
    cx, cy = 50.0, 40.0
    return Polygon(
        [
            (cx - hole_w / 2, cy - hole_h / 2),
            (cx + hole_w / 2, cy - hole_h / 2),
            (cx + hole_w / 2, cy + hole_h / 2),
            (cx - hole_w / 2, cy + hole_h / 2),
        ]
    )


# TS-09
# Verifies: MAP-5
def test_ts09_known_hole_coverage_in_expected_band():
    hole = _centered_hole(area_fraction=0.10)
    covered = REF_RECT.difference(hole)

    result = cov.compute_coverage(covered, REF_RECT, threshold=95.0)

    assert 88.0 <= result["coverage_pct"] <= 92.0
    assert result["verdict"] == "fail"


# TS-09
# Verifies: MAP-5
def test_ts09_full_polygon_passes_at_threshold():
    result = cov.compute_coverage(REF_RECT, REF_RECT, threshold=95.0)

    assert result["coverage_pct"] >= 99.9
    assert result["verdict"] == "pass"


def test_compute_coverage_empty_covered_region_is_zero():
    result = cov.compute_coverage(Polygon(), REF_RECT, threshold=95.0)
    assert result["coverage_pct"] == 0.0
    assert result["verdict"] == "fail"


def test_compute_coverage_disjoint_region_is_zero():
    far_away = Polygon([(1000, 1000), (1010, 1000), (1010, 1010), (1000, 1010)])
    result = cov.compute_coverage(far_away, REF_RECT, threshold=95.0)
    assert result["coverage_pct"] == 0.0
    assert result["verdict"] == "fail"


def test_compute_coverage_default_threshold_is_95():
    assert cov.DEFAULT_COVERAGE_THRESHOLD == 95.0
    result = cov.compute_coverage(REF_RECT, REF_RECT)
    assert result["threshold"] == 95.0


def test_survey_polygon_from_manifest():
    manifest = {"polygon": [[0.0, 0.0], [100.0, 0.0], [100.0, 80.0], [0.0, 80.0]]}
    poly = cov.survey_polygon_from_manifest(manifest)
    assert poly.area == 8000.0


def test_write_report_produces_parseable_json_and_md(tmp_path):
    result = cov.compute_coverage(REF_RECT, REF_RECT, threshold=95.0)
    paths = cov.write_report(
        tmp_path / "products" / "report",
        result,
        extra={"sync_error_stats": {"mean_ms": 1.0, "max_ms": 2.0, "count_over_50ms": 0, "frame_count": 5}},
    )

    assert paths["json"].is_file()
    assert paths["md"].is_file()

    payload = json.loads(paths["json"].read_text())
    assert payload["verdict"] == "pass"
    assert isinstance(payload["coverage_pct"], (int, float))
    assert payload["sync_error_stats"]["frame_count"] == 5

    md_text = paths["md"].read_text()
    assert "Coverage Report" in md_text
    assert "pass" in md_text
