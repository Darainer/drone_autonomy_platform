"""Tests for the checked-in F2 reference sample dataset (TP-002 F2, T3.4).

# NOTE: MAP-4/MAP-5/MAP-7 verification is already covered by the T3.2/T3.3
# unit + smoke tests (test_run_pipeline_check.py, test_run_pipeline_full.py,
# test_coverage.py); this test only asserts that the checked-in F2 fixture
# itself is valid and satisfies the check-mode coverage gate -- it carries
# NO new `Verifies:` marker (infra task, T3.4). Referenced by TP-002 F2
# (fixture definition), TS-08 (full-mode smoke uses a fixture like this one),
# and TS-10 (onboard check-mode budget test scales this same generator up to
# 1000 frames on Jetson HIL -- not exercised here, see
# tests/fixtures/generate_sample_dataset.py's module docstring).

`run_pipeline.py --mode check` writes `products/report/*` under the dataset
directory it's pointed at. To keep the checked-in fixture pristine (no
generated products/ ever committed), this test runs `--mode check` against a
tmp_path COPY of the fixture, never the checked-in directory itself.
"""
from __future__ import annotations

import json
import pathlib
import shutil
import subprocess
import sys

import verify_dataset as vd

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
RUN_PIPELINE = REPO_ROOT / "run_pipeline.py"
F2_FIXTURE = REPO_ROOT / "tests" / "fixtures" / "survey_ref_rect_sample"


def test_f2_fixture_exists_and_has_no_checked_in_products():
    assert F2_FIXTURE.is_dir(), f"F2 fixture missing: {F2_FIXTURE}"
    assert not (F2_FIXTURE / "products").exists(), (
        "F2 fixture must stay pristine -- no generated products/ checked in "
        f"under {F2_FIXTURE}"
    )


def test_f2_fixture_is_valid_dataset():
    # (a) verify_dataset() on the checked-in F2 -- valid, empty errors.
    errors = vd.verify_dataset(F2_FIXTURE)
    assert errors == [], f"F2 fixture failed DES-004 validation: {errors}"


def test_f2_fixture_check_mode_passes_coverage_gate(tmp_path):
    # (b) run_pipeline.py --mode check on a tmp COPY of F2 -- exit 0, verdict
    # "pass", coverage_pct >= 95 in products/report/coverage.json. Copying
    # avoids ever writing products/ into the checked-in fixture.
    dataset_copy = tmp_path / F2_FIXTURE.name
    shutil.copytree(F2_FIXTURE, dataset_copy)

    result = subprocess.run(
        [sys.executable, str(RUN_PIPELINE), "--mode", "check", "--dataset", str(dataset_copy)],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr

    coverage_json = json.loads((dataset_copy / "products" / "report" / "coverage.json").read_text())
    assert coverage_json["valid"] is True
    assert coverage_json["verdict"] == "pass"
    assert isinstance(coverage_json["coverage_pct"], (int, float))
    assert coverage_json["coverage_pct"] >= 95.0

    # The checked-in fixture itself must remain untouched by this run.
    assert not (F2_FIXTURE / "products").exists()
