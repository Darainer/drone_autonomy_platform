#!/usr/bin/env python3
"""
PostToolUse hook: run the C4 drift check right after an edit that could
cause it, for fast feedback instead of waiting for the Stop hook.

Reads the PostToolUse hook JSON payload from stdin, looks at
tool_input.file_path, and — only if it falls under one of the paths that
feed scripts/generate_c4.py — runs `python scripts/generate_c4.py --check`
from the repo root ($CLAUDE_PROJECT_DIR, not the process cwd: see
.claude/agents/wp-implementer.md / docs/workflow/README.md on
check_architecture_gap.py's sys.path[0] import needing repo-root cwd; the
same discipline is kept here for consistency and because generate_c4.py
also assumes repo-relative paths like src/, docs/architecture/c4/).

WP-W7: `should_check` tracks generate_c4.py's actual inputs, verified
against its source, not a guess at its shape. generate_c4.py is the
source of truth for this list — if its globs change, update should_check
to match, or the two will drift silently:
  - `parse_launch_remappings` (generate_c4.py ~line 123) globs
    `SRC_DIR.glob("*/launch/*.py")` (package-local launch files, e.g.
    src/foo/launch/foo.launch.py) plus `(REPO/"launch").glob("*.py")`
    (top-level launch/*.py) — so BOTH matter, not just top-level launch/.
  - `parse_sources` (generate_c4.py ~lines 178-179) walks
    `SRC_DIR.rglob("*")` and parses any file whose suffix is `.cpp` OR
    `.py`, EXCLUDING anything with "/launch/" in its path (those are
    handled by parse_launch_remappings above, not parsed as node
    sources). So `src/foo/foo_node.py` is a real trigger and `.hpp` is
    NOT — generate_c4.py never parses header files, so a `.hpp`-only
    edit cannot itself cause C4 drift; `.hpp` was dropped from the
    trigger list for that reason (a header change that also touches a
    .cpp file still triggers via the .cpp edit).
  - `msgs/**` is kept as a trigger though generate_c4.py does not read
    msgs/*.msg files directly (message type names come from the
    C++/Python source, not the .msg definitions) — kept per WP-W7 spec
    as a conservative signal for message-definition changes.

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

# Kept in sync with scripts/generate_c4.py's own inputs — see the module
# docstring above for the exact line references. NOT `.hpp`: generate_c4.py
# never parses header files, only .cpp and .py.
TRIGGER_SRC_EXTS = (".cpp", ".py")
TRIGGER_LAUNCH_PREFIX = "launch/"
TRIGGER_MSGS_PREFIX = "msgs/"
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
    # msgs/** — see module docstring: kept conservatively even though
    # generate_c4.py doesn't glob it directly.
    if rel.startswith(TRIGGER_MSGS_PREFIX):
        return True
    # launch/*.py — top-level launch files, read by parse_launch_remappings
    # via (REPO/"launch").glob("*.py").
    if rel.startswith(TRIGGER_LAUNCH_PREFIX) and rel.endswith(".py"):
        return True
    # src/**/*.cpp or src/**/*.py — every .cpp/.py anywhere under src/,
    # whether it's a node source parsed by parse_sources (which excludes
    # "/launch/" paths) or a package-local launch file matched by
    # parse_launch_remappings' `SRC_DIR.glob("*/launch/*.py")` (which is
    # exactly the "/launch/" paths parse_sources excludes) — between the
    # two, every .cpp/.py under src/ is read by one or the other.
    if rel.startswith(TRIGGER_SRC_PREFIX) and rel.endswith(TRIGGER_SRC_EXTS):
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
