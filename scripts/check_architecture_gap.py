#!/usr/bin/env python3
"""
Capability gap analysis: target architecture vs. code.

Reads target-architecture specs (docs/architecture/target/*.yaml) that declare
what a capability NEEDS — containers (ROS2 nodes, offboard modules), topic
flows, and behaviors — and diffs them against the CURRENT architecture parsed
from source by scripts/generate_c4.py. Writes one gap report per capability to
docs/reports/gap_<capability>.md.

Presence rules:
    container kind=node      present if a ROS2 node with that name parses from src/
    container kind=offboard  present if its `path` exists in the repo
    container kind=external  present if the alias is in generate_c4.EXTERNAL_SYSTEMS
    flow                     present if the current topic graph has edge
                             (from, to, topic) — remappings already applied
    behavior                 present if its `evidence` string (usually an
                             "Implements: <UID>" marker) is found under src/

Usage:
    python scripts/check_architecture_gap.py                # all target specs
    python scripts/check_architecture_gap.py docs/architecture/target/CAP-001-*.yaml
    python scripts/check_architecture_gap.py --strict       # exit 1 if gaps remain
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

import generate_c4

REPO = Path(__file__).resolve().parent.parent
TARGET_DIR = REPO / "docs" / "architecture" / "target"
REPORT_DIR = REPO / "docs" / "reports"

GENERATED_HEADER = (
    "<!-- GENERATED FILE — do not edit by hand. "
    "Regenerate with: python scripts/check_architecture_gap.py -->\n"
)


def load_current_model():
    nodes = generate_c4.parse_sources()
    edges, _rows, _warn, _dangling = generate_c4.build_edges(nodes)
    node_names = {n.name for n in nodes}
    edge_keys = {(src, dst, label) for src, dst, label, _tech, _kind in edges}
    externals = {alias for alias, _n, _d, _rx in generate_c4.EXTERNAL_SYSTEMS}
    return node_names, edge_keys, externals


def grep_src(needle: str) -> list[str]:
    hits = []
    for path in sorted((REPO / "src").rglob("*")):
        if path.suffix in (".cpp", ".hpp", ".py") and needle in path.read_text(errors="replace"):
            hits.append(str(path.relative_to(REPO)))
    return hits


def check_capability(spec_path: Path, node_names, edge_keys, externals):
    spec = yaml.safe_load(spec_path.read_text())
    cap = spec["capability"]
    rows: list[tuple[str, str, str, bool, str]] = []  # (section, name, req, present, detail)

    for c in spec.get("containers", []):
        kind = c.get("kind", "node")
        name, req = c["name"], c.get("requirement", "—")
        if kind == "node":
            present = name in node_names
            detail = "ROS2 node parsed from src/" if present else "no such node in src/"
        elif kind == "offboard":
            p = c.get("path", "")
            present = bool(p) and (REPO / p).exists()
            detail = f"`{p}` {'exists' if present else 'does not exist'}"
        elif kind == "external":
            present = name in externals
            detail = "registered in EXTERNAL_SYSTEMS" if present else \
                "not in generate_c4.EXTERNAL_SYSTEMS"
        else:
            present, detail = False, f"unknown kind `{kind}`"
        rows.append(("Containers", name, req, present, detail))

    for f in spec.get("flows", []):
        key = (f["from"], f["to"], f["topic"])
        present = key in edge_keys
        name = f"{f['from']} → {f['to']}"
        detail = f"`{f['topic']}`" + ("" if present else " — no such edge in current topic graph")
        rows.append(("Flows", name, f.get("requirement", "—"), present, detail))

    for b in spec.get("behaviors", []):
        evidence = b["evidence"]
        hits = grep_src(evidence)
        present = bool(hits)
        detail = f"`{evidence}` in {', '.join(hits)}" if present else \
            f"no `{evidence}` marker under src/"
        rows.append(("Behaviors", b["name"], b.get("requirement", "—"), present, detail))

    return spec, cap, rows


def render_report(spec, cap: str, rows, spec_path: Path) -> str:
    present_n = sum(1 for r in rows if r[3])
    total = len(rows)
    lines = [
        GENERATED_HEADER,
        f"# Capability Gap Report — {cap}: {spec.get('title', '')}",
        "",
        f"Target spec: `{spec_path.relative_to(REPO)}` · "
        f"Stakeholder requirements: {', '.join(spec.get('stakeholder_requirements', []))}",
        "",
        f"**{present_n} / {total}** target elements present — "
        + ("**capability complete**" if present_n == total
           else f"**{total - present_n} gap(s) remain**"),
    ]
    for section in ("Containers", "Flows", "Behaviors"):
        section_rows = [r for r in rows if r[0] == section]
        if not section_rows:
            continue
        lines += ["", f"## {section}", "",
                  "| Target element | Requirement | Status | Detail |", "|---|---|---|---|"]
        for _s, name, req, present, detail in section_rows:
            status = "✅ present" if present else "❌ missing"
            lines.append(f"| {name} | {req} | {status} | {detail} |")

    gaps = [r for r in rows if not r[3]]
    if gaps:
        lines += ["", "## Gap list (implementation handoff input)", ""]
        for _s, name, req, _p, _d in gaps:
            lines.append(f"- [ ] {name} ({req})")
        lines += ["", "Turn gaps into work: see the `capability` skill — each gap cluster "
                      "gets a design doc (`design` skill) and an agent work package.",]
    return "\n".join(lines) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("specs", nargs="*", help="target spec YAML files (default: all)")
    ap.add_argument("--strict", action="store_true", help="exit 1 if any gaps remain")
    args = ap.parse_args()

    spec_paths = [Path(s) for s in args.specs] or sorted(TARGET_DIR.glob("*.yaml"))
    if not spec_paths:
        sys.exit(f"error: no target specs found in {TARGET_DIR}")

    node_names, edge_keys, externals = load_current_model()
    total_gaps = 0
    for spec_path in spec_paths:
        spec, cap, rows = check_capability(spec_path.resolve(), node_names, edge_keys, externals)
        report = render_report(spec, cap, rows, spec_path.resolve())
        out = REPORT_DIR / f"gap_{cap}.md"
        REPORT_DIR.mkdir(parents=True, exist_ok=True)
        out.write_text(report)
        gaps = sum(1 for r in rows if not r[3])
        total_gaps += gaps
        print(f"{cap}: {len(rows) - gaps}/{len(rows)} present -> {out.relative_to(REPO)}")

    if args.strict and total_gaps:
        print(f"strict mode: {total_gaps} gap(s) remain", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
