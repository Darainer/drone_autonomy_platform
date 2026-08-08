#!/usr/bin/env python3
"""
PostToolUse hook: run the C4 drift check right after an edit that could
cause it, for fast feedback instead of waiting for the Stop hook.

Reads the PostToolUse hook JSON payload from stdin, looks at
tool_input.file_path, and — only if it falls under one of the paths that
feed scripts/generate_c4.py (src/**/*.cpp, src/**/*.hpp, launch/**,
msgs/**) — runs `python scripts/generate_c4.py --check` from the repo
root ($CLAUDE_PROJECT_DIR, not the process cwd: see
.claude/agents/wp-implementer.md / docs/workflow/README.md on
check_architecture_gap.py's sys.path[0] import needing repo-root cwd; the
same discipline is kept here for consistency and because generate_c4.py
also assumes repo-relative paths like src/, docs/architecture/c4/).

generate_c4.py prints staleness to stdout; we exit 2 (blocking — message
surfaced from stderr) on failure so the model sees it immediately, so we
copy the check's combined output onto our own stderr in that case.

Fails open on anything that isn't a match or isn't parseable: this hook
narrows to a known set of paths and must never block an unrelated edit.
"""
import json
import os
import subprocess
import sys

TRIGGER_EXTS = (".cpp", ".hpp")
TRIGGER_DIR_PREFIXES = ("launch/", "msgs/")
TRIGGER_SRC_PREFIX = "src/"


def repo_root() -> str:
    return os.environ.get("CLAUDE_PROJECT_DIR") or os.getcwd()


def normalized_relative_path(file_path: str, root: str) -> str | None:
    if not file_path:
        return None
    abs_path = file_path if os.path.isabs(file_path) else os.path.join(root, file_path)
    abs_path = os.path.normpath(abs_path)
    root = os.path.normpath(root)
    try:
        rel = os.path.relpath(abs_path, root)
    except ValueError:
        return None
    if rel.startswith(".."):
        return None
    return rel.replace(os.sep, "/")


def should_check(rel: str) -> bool:
    if rel.startswith(TRIGGER_DIR_PREFIXES):
        return True
    if rel.startswith(TRIGGER_SRC_PREFIX) and rel.endswith(TRIGGER_EXTS):
        return True
    return False


def main() -> int:
    try:
        payload = json.load(sys.stdin)
    except (json.JSONDecodeError, ValueError):
        return 0

    tool_input = payload.get("tool_input") or {}
    file_path = tool_input.get("file_path")
    if not file_path:
        return 0

    root = repo_root()
    rel = normalized_relative_path(file_path, root)
    if rel is None or not should_check(rel):
        return 0

    result = subprocess.run(
        [sys.executable, "scripts/generate_c4.py", "--check"],
        cwd=root,
        capture_output=True,
        text=True,
    )
    combined = (result.stdout or "") + (result.stderr or "")
    if result.returncode != 0:
        print(
            f"C4 views are stale after editing '{rel}':\n{combined}\n"
            "Run `python scripts/generate_c4.py` (repo root) and commit the "
            "regenerated views.",
            file=sys.stderr,
        )
        return 2  # blocking
    return 0


if __name__ == "__main__":
    sys.exit(main())
