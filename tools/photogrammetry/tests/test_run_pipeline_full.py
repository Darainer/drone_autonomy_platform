"""TS-08 smoke test for `run_pipeline.py --mode full` (DES-005 D1/D4/D6/D1-D6).

# TS-08
# Verifies: MAP-4

Real ODM needs Docker+GPU, unavailable in this environment/CI, so this test
drives `_mode_full` in-process with a FAKE low-level ODM runner injected at
the `odm_runner.run_odm(..., runner=...)` seam (via `run_pipeline._mode_full`'s
`odm_runner_fn` parameter, defaulted to `odm_runner.run_odm` for real use).
Using the real `run_odm()` orchestration with only the low-level
`runner(argv) -> CompletedProcess` faked means geo.txt generation,
`build_odm_command()` argv construction, AND the product COLLECTION step
(normalizing ODM's per-stage subfolders into products/) are exercised for
real; only the actual `docker run` invocation is stubbed. The fake runner
writes ODM's REAL output layout (odm_georeferencing/, odm_texturing/,
odm_orthophoto/ subfolders) so run_odm's collection is not bypassed, plus an
ENU reconstruction-footprint sidecar covering the survey polygon. The test
asserts the full TS-08 contract: exit 0; products/ contains non-empty
.laz/.obj(+.mtl)/.tif; report/coverage.json has a numeric coverage_pct; and
no interactive prompt is ever emitted (builtins.input is patched to raise if
called at all, enforcing STK-1(d) zero-interactive-input end to end).
"""
from __future__ import annotations

import functools
import json
import subprocess

import run_pipeline
from photogrammetry import odm_runner

# Default dataset_factory manifest polygon (see conftest.build_dataset): a
# 10x10 square in local ENU (x, y). The fake reconstruction footprint below
# is a superset of it, so post-reconstruction coverage QA passes at the
# default 95% threshold.
_COVERING_FOOTPRINT_VERTICES = [(-5.0, -5.0), (15.0, -5.0), (15.0, 15.0), (-5.0, 15.0)]


def _write_odm_output_layout(dataset_dir):
    """Write ODM's REAL per-stage output subfolders under the dataset dir.

    Mirrors what a genuine `docker run opendronemap/odm ...` leaves behind, so
    run_odm's collection step (odm_* subfolders -> products/) is exercised end
    to end rather than bypassed by writing products/ directly.
    """
    (dataset_dir / "odm_georeferencing").mkdir(parents=True, exist_ok=True)
    (dataset_dir / "odm_georeferencing" / "odm_georeferenced_model.laz").write_bytes(b"stub-laz")
    (dataset_dir / "odm_texturing").mkdir(parents=True, exist_ok=True)
    (dataset_dir / "odm_texturing" / "odm_textured_model_geo.obj").write_bytes(b"stub-obj")
    (dataset_dir / "odm_texturing" / "odm_textured_model_geo.mtl").write_bytes(b"stub-mtl")
    (dataset_dir / "odm_texturing" / "odm_textured_model_geo_material0000.png").write_bytes(b"tex")
    (dataset_dir / "odm_orthophoto").mkdir(parents=True, exist_ok=True)
    (dataset_dir / "odm_orthophoto" / "odm_orthophoto.tif").write_bytes(b"stub-tif")


def _make_fake_docker_runner(dataset_dir, footprint_vertices):
    """A fake `runner(argv) -> CompletedProcess` standing in for real ODM.

    Writes ODM's real output subfolders (so collection runs) plus an ENU
    reconstruction-footprint sidecar. Asserts the argv it receives is
    non-interactive (TS-08 "argv has no -i/-t/-it").
    """

    def _fake_runner(argv):
        disallowed = {"-i", "-t", "-it", "-ti", "--interactive", "--tty"}
        assert not (disallowed & set(argv)), f"interactive flag in argv: {argv}"
        _write_odm_output_layout(dataset_dir)
        odm_runner.write_footprint_sidecar(dataset_dir, footprint_vertices)
        return subprocess.CompletedProcess(argv, returncode=0, stdout="", stderr="")

    return _fake_runner


def test_ts08_mode_full_smoke_with_fake_odm(dataset_factory, monkeypatch, capsys):
    dataset_dir = dataset_factory(mission_id="ts08_full_smoke")

    # STK-1(d) enforcement: if _mode_full (or anything it calls) ever
    # solicits interactive input, fail loudly instead of hanging/blocking.
    def _no_input(*args, **kwargs):
        raise AssertionError("run_pipeline --mode full must never call input() (STK-1(d))")

    monkeypatch.setattr("builtins.input", _no_input)

    # products/ must not exist before the run -- proves the collection step
    # (odm_* subfolders -> products/) is what populates it.
    assert not (dataset_dir / "products").exists()

    fake_low_level_runner = _make_fake_docker_runner(dataset_dir, _COVERING_FOOTPRINT_VERTICES)
    odm_runner_fn = functools.partial(odm_runner.run_odm, runner=fake_low_level_runner)

    exit_code = run_pipeline._mode_full(
        str(dataset_dir),
        polygon_from_manifest=True,
        min_coverage=95.0,
        odm_runner_fn=odm_runner_fn,
    )

    captured = capsys.readouterr()

    # Pass: exit 0.
    assert exit_code == 0, captured.out + captured.err

    # Pass: products/ contains non-empty .laz, .obj (+material), .tif --
    # collected from ODM's real output subfolders by run_odm.
    products = odm_runner.collect_products(dataset_dir)
    for key in ("laz", "obj", "mtl", "orthophoto"):
        path = products[key]
        assert path.is_file(), f"missing product: {path}"
        assert path.stat().st_size > 0, f"empty product: {path}"

    # Pass: report/coverage.json has a numeric coverage_pct.
    report_dir = dataset_dir / "products" / "report"
    coverage_json = json.loads((report_dir / "coverage.json").read_text())
    assert isinstance(coverage_json["coverage_pct"], (int, float))
    assert coverage_json["verdict"] == "pass"
    assert coverage_json["mode"] == "full"
    assert coverage_json["products_complete"] is True
    assert (report_dir / "coverage.md").is_file()

    # Pass: NO prompt ever on stdout. There is no universal definition of
    # "a prompt", but a fully unattended pipeline that never calls input()
    # (enforced above) and only emits the status lines this module prints
    # itself gives a concrete, checkable proxy: no interrogative/
    # confirmation phrasing anywhere in stdout.
    lowered = captured.out.lower()
    for forbidden in ("(y/n)", "[y/n]", "press enter", "continue?", "y/n?"):
        assert forbidden not in lowered, f"prompt-like text found in stdout: {forbidden!r}"


def test_mode_full_invalid_dataset_fails_fast_without_invoking_odm(dataset_factory):
    """Check stage runs FIRST; an invalid dataset must never reach ODM."""
    dataset_dir = dataset_factory(mission_id="ts08_invalid", finalize_manifest=False)

    odm_was_called = {"value": False}

    def _odm_runner_fn(*args, **kwargs):
        odm_was_called["value"] = True
        raise AssertionError("ODM must not be invoked for an invalid dataset")

    exit_code = run_pipeline._mode_full(
        str(dataset_dir),
        polygon_from_manifest=True,
        min_coverage=95.0,
        odm_runner_fn=_odm_runner_fn,
    )

    assert exit_code != 0
    assert odm_was_called["value"] is False

    report_dir = dataset_dir / "products" / "report"
    coverage_json = json.loads((report_dir / "coverage.json").read_text())
    assert coverage_json["valid"] is False
    assert coverage_json["coverage_pct"] is None


def test_mode_full_under_coverage_exits_nonzero(dataset_factory):
    """Products complete but reconstruction footprint under-covers the survey polygon -> fail."""
    dataset_dir = dataset_factory(mission_id="ts08_under_coverage")

    # A tiny sliver of the manifest's 10x10 survey polygon -- well under the
    # 95% coverage gate. Products still collected from ODM's real layout.
    under_covering = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    fake_runner = _make_fake_docker_runner(dataset_dir, under_covering)
    odm_runner_fn = functools.partial(odm_runner.run_odm, runner=fake_runner)

    exit_code = run_pipeline._mode_full(
        str(dataset_dir),
        polygon_from_manifest=True,
        min_coverage=95.0,
        odm_runner_fn=odm_runner_fn,
    )

    assert exit_code != 0

    report_dir = dataset_dir / "products" / "report"
    coverage_json = json.loads((report_dir / "coverage.json").read_text())
    assert coverage_json["verdict"] == "fail"
    assert coverage_json["coverage_pct"] < 95.0
