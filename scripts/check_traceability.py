#!/usr/bin/env python3
"""
Requirements traceability checker and matrix generator.

Parses requirement UIDs from docs/requirements/*.sdoc (StrictDoc format) and
scans the repository for linkage markers:

    // Implements: COMP-3, SAF-1      (source code — C++/Python comments)
    # Verifies: PLAT-1                (test code)
    bare UID mentions                 (docs/design, docs/architecture, docs/test_plans)

Emits a traceability matrix to docs/reports/traceability_matrix.md.

Usage:
    python scripts/check_traceability.py             # write matrix
    python scripts/check_traceability.py --stdout    # print instead of write
    python scripts/check_traceability.py --strict    # exit 1 if any Approved requirement
                                                     # lacks an implemented Verifies: test
                                                     # (a test-plan mention alone does not count)
"""

from __future__ import annotations

import argparse
import re
import sys
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
REQ_DIR = REPO / "docs" / "requirements"
OUT_FILE = REPO / "docs" / "reports" / "traceability_matrix.md"

UID_RX = re.compile(r"\b([A-Z][A-Z0-9]{1,15}-\d+)\b")
MARKER_RX = re.compile(r"(Implements|Verifies):\s*([A-Z0-9\-,\s]+)")

CODE_GLOBS = ["src/**/*.cpp", "src/**/*.hpp", "src/**/*.py", "agents/**/*.py",
              "scripts/**/*.py", "launch/**/*.py"]
DOC_GLOBS = ["docs/design/**/*.md", "docs/architecture/**/*.md",
             "docs/test_plans/**/*.md", "docs/standards/**/*.md"]

GENERATED_HEADER = (
    "<!-- GENERATED FILE — do not edit by hand. "
    "Regenerate with: python scripts/check_traceability.py -->\n"
)


def parse_sdoc_requirements() -> dict[str, dict]:
    """Light-weight .sdoc parse: UID/STATUS/TITLE/TAGS of each [REQUIREMENT]."""
    reqs: dict[str, dict] = {}
    if not REQ_DIR.exists():
        sys.exit(f"error: {REQ_DIR} does not exist — see the `requirements` skill")
    for path in sorted(REQ_DIR.glob("*.sdoc")):
        current: dict | None = None
        for line in path.read_text().splitlines():
            stripped = line.strip()
            if stripped == "[REQUIREMENT]":
                current = {"file": path.relative_to(REPO), "status": "Draft",
                           "title": "", "tags": ""}
                continue
            if stripped.startswith("[") and stripped.endswith("]"):
                current = None
                continue
            if current is None:
                continue
            m = re.match(r"^(UID|STATUS|TITLE|TAGS):\s*(.*)$", line)
            if not m:
                continue
            key, val = m.group(1).lower(), m.group(2).strip()
            if key == "uid":
                if val in reqs:
                    sys.exit(f"error: duplicate UID {val} in {path.name}")
                reqs[val] = current
            current[key] = val
    return {uid: r for uid, r in reqs.items() if "uid" in r}


def iter_files(globs: list[str]):
    for pattern in globs:
        for path in sorted(REPO.glob(pattern)):
            if path.is_file() and "docs/reports" not in str(path):
                yield path


def scan_markers(known_uids: set[str]):
    """Return per-UID refs: implements (code), verifies (tests), docs mentions."""
    implements = defaultdict(set)
    verifies = defaultdict(set)
    doc_refs = defaultdict(set)
    unknown: set[tuple[str, str]] = set()

    for path in iter_files(CODE_GLOBS):
        if path.name == Path(__file__).name or path.name == "generate_c4.py":
            continue
        rel = str(path.relative_to(REPO))
        is_test = "test" in path.name or "tests" in path.parts
        for kind, uids in MARKER_RX.findall(path.read_text(errors="replace")):
            for uid in UID_RX.findall(uids):
                if uid not in known_uids:
                    unknown.add((uid, rel))
                    continue
                if kind == "Verifies" and is_test:
                    verifies[uid].add(rel)
                elif kind == "Verifies":
                    # Verifies marker outside a test file still counts, but is odd
                    verifies[uid].add(rel + " (not a test file)")
                else:
                    implements[uid].add(rel)

    for path in iter_files(DOC_GLOBS):
        rel = str(path.relative_to(REPO))
        text = path.read_text(errors="replace")
        for uid in set(UID_RX.findall(text)):
            if uid in known_uids:
                doc_refs[uid].add(rel)

    return implements, verifies, doc_refs, unknown


def uid_sort_key(uid: str):
    family, _, num = uid.rpartition("-")
    return (family, int(num))


def build_matrix(reqs, implements, verifies, doc_refs) -> tuple[str, list[str]]:
    lines = [
        GENERATED_HEADER,
        "# Requirements Traceability Matrix",
        "",
        "| UID | Title | Status | Design/Docs | Code (Implements) | Tests (Verifies) | Coverage |",
        "|---|---|---|---|---|---|---|",
    ]
    # Requirements this file marks "Approved" but that have no *implemented*
    # Verifies: test marker — includes both "planned" (test plan exists, no
    # test yet) and "uncovered" (nothing at all). Only "✅ verified" satisfies
    # an Approved requirement for --strict; "planned" is not enough.
    approved_without_test: list[str] = []
    counts = defaultdict(int)

    for uid in sorted(reqs, key=uid_sort_key):
        r = reqs[uid]
        docs = "<br>".join(sorted(doc_refs.get(uid, []))) or "—"
        code = "<br>".join(sorted(implements.get(uid, []))) or "—"
        tests = "<br>".join(sorted(verifies.get(uid, []))) or "—"
        if verifies.get(uid):
            coverage = "✅ verified"
        elif doc_refs.get(uid) and any("test_plans" in d for d in doc_refs[uid]):
            coverage = "🟡 planned"
        else:
            coverage = "❌ uncovered"
        if coverage != "✅ verified" and r["status"].lower() == "approved":
            approved_without_test.append(uid)
        counts[coverage.split()[1]] += 1
        safety = " ⚠" if "safety-critical" in r.get("tags", "") else ""
        lines.append(
            f"| {uid}{safety} | {r['title']} | {r['status']} | {docs} | {code} | {tests} | {coverage} |"
        )

    total = len(reqs)
    lines += [
        "",
        "## Summary",
        "",
        f"- Requirements: **{total}**",
        f"- Verified by test: **{counts['verified']}**",
        f"- Verification planned (test plan only): **{counts['planned']}**",
        f"- Uncovered: **{counts['uncovered']}**",
        "",
        "⚠ = tagged `safety-critical` (DO-178C review scope, see "
        "docs/standards/do_178c_context.md)",
        "",
        "Linkage markers: `Implements: <UID>` in source, `Verifies: <UID>` in tests, "
        "bare UID mentions in design docs and test plans.",
    ]
    return "\n".join(lines) + "\n", approved_without_test


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--stdout", action="store_true", help="print matrix to stdout")
    ap.add_argument("--strict", action="store_true",
                    help="exit 1 if any Approved requirement lacks an implemented "
                         "Verifies: test marker (a test-plan mention alone does not count)")
    args = ap.parse_args()

    reqs = parse_sdoc_requirements()
    if not reqs:
        sys.exit("error: no requirements found in docs/requirements/*.sdoc")
    implements, verifies, doc_refs, unknown = scan_markers(set(reqs))
    matrix, approved_without_test = build_matrix(reqs, implements, verifies, doc_refs)

    if args.stdout:
        print(matrix)
    else:
        OUT_FILE.parent.mkdir(parents=True, exist_ok=True)
        OUT_FILE.write_text(matrix)
        print(f"wrote {OUT_FILE.relative_to(REPO)}")

    for uid, where in sorted(unknown):
        print(f"warning: marker references unknown UID {uid} in {where}", file=sys.stderr)

    print(f"{len(reqs)} requirements; {len(approved_without_test)} approved without "
          "verified test coverage")
    if args.strict and approved_without_test:
        print("strict mode: approved requirements without an implemented Verifies: "
              "test: " + ", ".join(approved_without_test), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
