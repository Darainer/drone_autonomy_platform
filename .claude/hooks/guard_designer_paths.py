#!/usr/bin/env python3
"""
PreToolUse hook: block Edit|Write on designer-owned paths.

docs/architecture/target/** and docs/capabilities/** are owned by the
`capability` skill / designer role. Implementation sessions (Claude Code
working directly, or wp-implementer work packages) must never edit them —
see CLAUDE.md and .claude/agents/wp-implementer.md. This used to be a
sentence any session could forget; here it is a gate.

Reads the PreToolUse hook JSON payload from stdin, looks at
tool_input.file_path (Edit/Write shape), and:
  - exits 2 (blocking, message on stderr) if the path falls under a
    designer-owned prefix and the designer exemption (below) is not active,
  - exits 0 (allow) otherwise, including when the payload doesn't parse or
    carries no file_path (fail open — this hook only ever narrows a small,
    known set of paths, it must never be the reason an unrelated edit fails).

Path-independent: resolves the repo root from $CLAUDE_PROJECT_DIR (set by
Claude Code for every hook invocation) rather than the process cwd.

Designer exemption (WP-W7): `.claude/skills/capability/SKILL.md` steps 2-3
instruct the designer to write exactly these two prefixes, so a
zero-exception guard blocks the very workflow that owns them. The exemption
is the environment variable DAP_DESIGNER_SESSION — set to 1/true/yes
(case-insensitive) to stand the guard down; anything else (unset, empty,
"0", or any other value) leaves the guard blocking. This is deliberately an
environment variable, not a hook argument or an in-session flag: a running
session cannot alter the environment it already inherited, so granting the
exemption takes a deliberate human act at launch
(`DAP_DESIGNER_SESSION=1 claude`) rather than something a model could grant
itself mid-task. When active, the guard still prints a stand-down note to
stderr on every exempted write — silence would hide that the exemption
fired.
"""
import json
import os
import sys

# Repo-relative prefixes that must never be touched by an implementation
# session. Matched as directory prefixes (anything under them), not exact
# files, since "**/*" in the WP spec means the whole subtree.
BLOCKED_PREFIXES = (
    "docs/architecture/target/",
    "docs/capabilities/",
)


DESIGNER_SESSION_TRUE_VALUES = {"1", "true", "yes"}


def designer_session_active() -> bool:
    """DAP_DESIGNER_SESSION=1/true/yes (case-insensitive) stands the guard
    down. Anything else — unset, empty, "0", or any other value — leaves it
    blocking. See the module docstring for why this is an env var."""
    return os.environ.get("DAP_DESIGNER_SESSION", "").strip().lower() in DESIGNER_SESSION_TRUE_VALUES


def repo_root() -> str:
    return os.environ.get("CLAUDE_PROJECT_DIR") or os.getcwd()


def normalized_relative_path(file_path: str, root: str) -> str | None:
    """Return file_path relative to root with forward slashes, or None if
    it can't be resolved (e.g. lives outside the repo)."""
    if not file_path:
        return None
    abs_path = file_path if os.path.isabs(file_path) else os.path.join(root, file_path)
    abs_path = os.path.normpath(abs_path)
    root = os.path.normpath(root)
    try:
        rel = os.path.relpath(abs_path, root)
    except ValueError:
        return None  # different drive on Windows, etc. — not our concern here
    if rel.startswith(".."):
        return None  # outside the repo entirely
    return rel.replace(os.sep, "/")


def main() -> int:
    try:
        payload = json.load(sys.stdin)
    except (json.JSONDecodeError, ValueError):
        return 0  # fail open: can't inspect it, don't block it

    tool_input = payload.get("tool_input") or {}
    file_path = tool_input.get("file_path") or tool_input.get("notebook_path")
    if not file_path:
        return 0

    rel = normalized_relative_path(file_path, repo_root())
    if rel is None:
        return 0

    for prefix in BLOCKED_PREFIXES:
        if rel == prefix.rstrip("/") or rel.startswith(prefix):
            if designer_session_active():
                print(
                    f"STAND-DOWN: '{rel}' is designer-owned (matches "
                    f"'{prefix}**'), but DAP_DESIGNER_SESSION exempts this "
                    "session — allowing the write.",
                    file=sys.stderr,
                )
                return 0  # exemption active: allow, but say so
            print(
                f"BLOCKED: '{rel}' is designer-owned (matches '{prefix}**').\n"
                "docs/architecture/target/** and docs/capabilities/** are never "
                "edited by implementation sessions — target/capability changes "
                "go back to the designer via the `capability` skill. "
                "See .claude/skills/capability/SKILL.md and "
                ".claude/agents/wp-implementer.md. "
                "If this session IS the designer, launch it with "
                "DAP_DESIGNER_SESSION=1 set (e.g. `DAP_DESIGNER_SESSION=1 "
                "claude`) instead of editing around this guard.",
                file=sys.stderr,
            )
            return 2  # blocking

    return 0


if __name__ == "__main__":
    sys.exit(main())
