# Claude Code workflow review — findings

**Date:** 2026-08-08
**Scope:** the repo's Claude Code setup — `CLAUDE.md`, `.claude/skills/`, the
checker scripts, CI, and the contribution/safety documentation.

> **Provenance.** This document is a synthesis, not a verbatim review. It
> reconstructs findings F1–F8 from the review that prompted this work, and merges
> in corrections established by running the code on 2026-08-08. Where the original
> review and the code disagreed, the code won and the correction is recorded
> inline. Work packages cite finding IDs from this page.

---

## Premise

The process this repo describes is sound. The problem is where it lives: it is
prose that an agent is asked to remember, rather than gates it cannot skip. Every
"standing rule" in `CLAUDE.md` and every enforcement claim in `CONTRIBUTING.md`
depends on a model choosing to comply, and nothing detects non-compliance.

A second, sharper problem sits on top of it. Claude Code is the implementer in
practice, but `CLAUDE.md` opens by instructing every session to route real work
into the Temporal prototype under `agents/`. The first thing a session reads is
an instruction to hand its work to a system that does not run.

---

## F1 — `CLAUDE.md` is the workforce manual, not the session guide

`CLAUDE.md` opens with *My Role* ("I am the orchestrator… submit it to the agent
workforce via `scripts/submit_task.py`"), followed by the plan schema, the agent
and task-queue table, the LLM backend table, and the rework loop. Roughly the
first 72 of 149 lines instruct the session to delegate into `agents/`.

That prototype is real but untested as a working loop, and the file gives no hint
of its status. Worse, the entry point is documented inconsistently: `CLAUDE.md`
teaches `scripts/submit_task.py`, while `agents/README.md` and CI both use
`scripts/task.sh` — `submit_task.py` runs inside the orchestrator container.

The material a session actually needs — how to build, how to test, what is
off-limits, what counts as verification — is either absent or below the fold.

→ **WP-W1**

## F2 — The safety gate is documentation only

`CONTRIBUTING.md` states that "the agent workflow automatically detects these
paths and applies a DO-178C review checklist plus a mandatory human approval gate
before any deploy." All three parts are false:

- **No path detection exists anywhere in the repo.** The Temporal gate keys on a
  `safety_critical` boolean that an LLM populates from a prompt. Nothing inspects
  a diff, a file list, or a path.
- **The "DO-178C checklist" is one line of prompt text** — `4. If safety-critical:
  apply DO-178C checklist` — with no checklist content supplied to the model.
- **The gate is bypassed by its only caller.** `scripts/submit_task.py` hardcodes
  `auto_approve=True`, and exposes no flag to turn it off.

Three further gaps compound it. `src/navigation/` is listed as safety-critical in
`CONTRIBUTING.md` but appears in no mechanism and is omitted from
`docs/standards/do_178c_context.md`. `CONTRIBUTING.md` also claims CI runs the
smoke test; it does not — `scripts/smoke_test.sh` is invoked by nothing, while
contributors are told to keep its node list current. And `.github/agents/`
contains three files (`safety_review.md`, `issue_triage.md`, `test_generation.md`)
that each describe a trigger no workflow implements; they are inert markdown that
reads as configuration.

*Correction to the original review, which flagged the phantom trigger in one file:
it is in four locations once `agents/README.md`'s safety-critical legend is
counted.*

→ **WP-W2**

## F3 — The standing rules are unenforced, and one of them cannot self-correct

`CLAUDE.md` lists five standing rules: regenerate C4 views after node/topic
changes, regenerate the traceability matrix after requirement changes, re-run the
gap check after capability merges, define requirement UIDs only in `.sdoc` files,
and never edit target specs or capability docs from an implementation session.

None of the three checker scripts runs in CI. None runs on a hook. Compliance is
entirely a matter of the model remembering, mid-task, to run a script.

**F3a.** Even if a gate were installed, one checker could not drive a fix.
`check_architecture_gap.py` prints only `strict mode: N gap(s) remain` — no
element names. The missing elements are computed and written into the report file,
but never reach stderr, so a hook or CI log cannot say what to fix. The other two
checkers already print specifics.

**F3b.** The repo has no `.claude/settings.json` at all — no hooks, no permissions
allowlist. Every checker invocation prompts for permission.

*Corrections to the original review:*
- The three scripts do **not** share a flag vocabulary. `generate_c4.py` takes
  `--check` and `--no-render`; the other two take `--strict`. The wrong flag exits 2.
- A Stop hook running all three **strictly** would block every turn from the moment
  it is installed: `check_traceability.py --strict` exits 1 today on 17 Approved
  requirements with no verifying test (`COMP-1..9`, `E2E-1..3`, `PLAT-1..3`,
  `SAF-1`, `SAF-2`). The traceability check must be advisory until those are covered.
- Both `check_*` scripts rewrite their report files as a side effect of checking,
  so a hook running them is not read-only.
- `generate_c4.py` prints staleness to stdout; the other two print to stderr. A
  hook capturing one stream loses half the diagnostics.

→ **WP-W3** (F3a), **WP-W4** (F3b)

## F4 — There is nothing to catch a wrong implementation

Out of scope for this round, recorded because it bounds what the other findings
can achieve.

`-DBUILD_TESTING=OFF` in all three Dockerfiles and in the documented build means
no C++ test ever compiles. `pyproject.toml` pins `testpaths = ["agents/tests"]`,
so the nine `tools/photogrammetry` test files and `src/mapping`'s replay test are
never collected. Three gtest files exist and never build. CI runs `pytest
agents/tests/` in mock mode and a Docker build, nothing else. `ruff` is configured
and invoked nowhere. `run_simulation` is an LLM call that runs no simulator and
asserts nothing; there is no SITL harness, no worlds, no PX4 launcher.

Meanwhile `src/control/` is a 34-line node that logs on receipt and never
publishes to the topic it advertises, and `src/safety/` is one genuine node
(`battery_monitor.cpp`, carrying the repo's only `Implements:` marker for SAF-1
and SAF-2) plus a stub. Neither package has tests.

The consequence is worth stating plainly: the safety gate guards code that is
largely unimplemented and entirely unverified, and no automation would notice a
plausible-but-wrong implementation on those paths.

## F5 — There is no review gate on generated work

Work lands with no independent check against its own spec. A session that
implements a change also decides whether the change is correct, which is the same
context judging itself.

The fix is a reviewer with a fresh context, given only the spec and the diff, that
returns an explicit verdict — and a driver that commits only on approval, capped
so a reviewer manufacturing findings cannot spin forever.

→ **WP-W0** (`design-conformance`, `wp-implementer`, `/work-package`)

## F6 — No model routing

All seven skills carry `name` and `description` only. Nothing distinguishes work
where a wrong answer is expensive and slow to notice (capability, design,
requirements, test plans) from work whose output a script verifies immediately
(C4 views, reports).

*Correction to the original review:* `model:` is a valid frontmatter key, but
`effort:` is not — the installed CLI parses no such key. And the alias list is
exactly `sonnet`, `opus`, `haiku`, `best`, `sonnet[1m]`, `opus[1m]`, `opusplan`;
anything else is passed through as a literal model ID.

→ **WP-W5**, routing table in [`docs/workflow/README.md`](../workflow/README.md)

## F7 — Generating skills are model-invocable

`c4` and `report` both write generated artifacts into the repo. Both can be
triggered by the model on its own initiative from a description match. Skills with
side effects should require explicit invocation.

→ **WP-W5**

## F8 — Session practice is undocumented

The habits that make the enforceable parts work — one session per work package,
clearing between them, restating rather than correcting a third time, spec before
implementation, worktrees for parallel work — exist only as practice. They are not
code and do not belong in `CLAUDE.md`, but they need one referenceable page.

→ **WP-W0**, [`docs/workflow/README.md`](../workflow/README.md)

---

## Finding not in the original review

**The C4 render toolchain is not provisioned.** `python scripts/generate_c4.py`
with no flags hard-exits when Graphviz or PlantUML is missing, so the standing rule
to regenerate and commit views is unrunnable on a clean machine.
`scripts/setup_c4_tooling.sh` exists but runs `apt-get install` without an
`apt-get update` and refuses to proceed without a pre-installed JRE.

This also weakens the gate itself: with a renderer present, `--check` byte-compares
the SVGs; without one, it only asserts the files exist. Regenerating with
`--no-render` therefore updates the `.md`/`.puml` outputs while leaving stale SVGs
that no check would catch.

→ **WP-W6**

---

## Verified baselines (2026-08-08)

| Command | Exit | Output |
|---|---|---|
| `python scripts/generate_c4.py --check --no-render` | 0 | `C4 views are up to date.` |
| `python scripts/check_architecture_gap.py --strict` | 0 | `CAP-001: 18/18 present` |
| `python scripts/check_traceability.py` | 0 | `28 requirements; 17 approved without verified test coverage` |
| `python scripts/check_traceability.py --strict` | **1** | names all 17 uncovered requirements |

Work packages, ordering, and session habits: [`docs/workflow/README.md`](../workflow/README.md).
