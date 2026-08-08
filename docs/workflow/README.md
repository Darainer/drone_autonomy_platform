# Claude Code workflow — work packages and session habits

How Claude Code sessions are run against this repo: the open work packages, where
each kind of task is routed, and the session habits that are not enforceable as
code.

Source: [2026-08-08 workflow review](../reports/2026-08-08-claude-code-workflow-review.md).

---

## Work packages

| WP | Finding | Status |
|---|---|---|
| WP-W0 | Loop scaffolding — reviewer/implementer agents, `/work-package` driver, this page | done |
| WP-W1 | F1 — CLAUDE.md is not the workforce manual | pending |
| WP-W2 | F2 — safety gate: warn-only CI, PR template, honest docs | pending |
| WP-W3 | F3a — gap checker failure output is not actionable | pending |
| WP-W6 | C4 render toolchain is not provisioned | pending |
| WP-W4 | F3b — hooks + `.claude/settings.json` | pending |
| WP-W5 | F7/F6 — skill hygiene and model routing | pending |

Order matters twice: **W3 before W4** (a Stop hook is only useful if its failure
output says what to fix), and **W6 before W4** (the C4 hook should gate on a real
byte comparison, and any SVG churn from installing the renderer must be settled
before the hook goes in, not discovered by it).

Out of scope for this round, deliberately: F4 in full — unit tests, `colcon test`
in CI, the `pyproject.toml` `testpaths` pin, `BUILD_TESTING=OFF`, and the SITL
harness. Consequence worth knowing: `src/control/` and `src/safety/` remain
largely stubs with no tests, SAF-1 and SAF-2 stay unverified, and nothing catches
a plausible-but-wrong implementation on those paths — so do not run long
unattended sessions against autonomy or control code yet.

---

### WP-W1 — CLAUDE.md is not the workforce manual

Move `CLAUDE.md` sections *My Role*, *Submitting a Task*, *Plan Schema*, *Valid
Agents and Task Queues*, *LLM Backend*, and *Rework Loop* into `agents/README.md`.
Nothing is deleted — they document a real prototype, just not the working loop.

- Reconcile rather than duplicate: `agents/README.md` already carries a task-queue
  table and an LLM-backend table that contradict CLAUDE.md's. Merge to one of each.
  Resolve the entry-point contradiction in favour of `scripts/task.sh` (what CI
  uses), noting that `submit_task.py` runs inside the orchestrator container.
- Add a dated status line at the top of `agents/README.md`: untested prototype,
  not part of the working loop as of 2026-08. Undated prototypes read as live systems.
- Rewrite `CLAUDE.md` to: build/test commands, the three-loop overview with the
  skill table, boundaries, verification-by-evidence, and adding a new node. State
  the test and lint situation honestly — the only automated test run is
  `pytest agents/tests/`, `-DBUILD_TESTING=OFF` means no C++ test ever builds, and
  `ruff` is configured but invoked nowhere. Do not invent commands.
- Keep the standing-rules prose for now. WP-W4 removes it once the hooks exist.

**Exit criteria**
- `wc -l CLAUDE.md` under 100.
- `grep -c submit_task CLAUDE.md` is 0.
- `agents/README.md` carries every moved section plus the dated status line, with
  no duplicated or contradicting table.

---

### WP-W2 — safety gate: warn-only CI, PR template, honest docs

- New job in `.github/workflows/ci.yml`: on pull request, diff against the base;
  if any path under `src/control/`, `src/safety/`, or `src/navigation/` changed
  and the PR lacks a `safety-reviewed` label, write a warning naming the changed
  files to `$GITHUB_STEP_SUMMARY` — and **exit 0**. The signal exists and the
  checklist gets filled in; nothing blocks until branch protection is configured.
- New `.github/pull_request_template.md` with a DO-178C checklist sourced from
  `docs/standards/do_178c_context.md`: DAL assignment and justification,
  safety-path declaration, objective-satisfaction rationale, verification evidence
  proportional to DAL, `Implements:`/`Verifies:` traceability, C4 drift clean.
- Correct every claim of enforcement that does not exist:
  - `CONTRIBUTING.md` — no path detection exists anywhere in the repo; the
    "DO-178C review checklist" is a single line of LLM prompt text with no
    checklist content; the human gate is bypassed because `scripts/submit_task.py`
    hardcodes `auto_approve=True` with no override flag. Mark the automated gate
    *planned*. Separately, CI does not run `scripts/smoke_test.sh` — and the
    instruction to maintain that script's node list points at automation that
    never runs.
  - `.github/agents/safety_review.md`, `issue_triage.md`, `test_generation.md` —
    all three describe triggers no workflow implements. A banner marking them
    inert is sufficient.
  - `agents/README.md` — the legend claiming safety-critical paths trigger a
    DO-178C review and human approval gate.
  - `docs/standards/do_178c_context.md` — add `src/navigation` so all locations
    agree on the safety-path list.

**Exit criteria**
- `ci.yml` parses as YAML.
- No sentence in `CONTRIBUTING.md`, `.github/agents/`, `agents/README.md`, or
  `docs/standards/` asserts automatic enforcement that does not exist.

**Human follow-up, not doable from a session:** create the `safety-reviewed`
label, and mark the check required in branch protection when you want it blocking.

---

### WP-W3 — make the gap checker's failure output actionable

`scripts/check_architecture_gap.py` prints only `strict mode: N gap(s) remain` on
failure. The missing element names are already computed and written into the
report's gap list; they just never reach stderr, so a hook or CI log cannot say
what to fix.

- Under `--strict`, enumerate the missing containers/flows/behaviours per
  capability, following the pattern already used in `check_traceability.py` and
  `generate_c4.py`. Print the report path alongside.
- Record the baseline: `--strict` exits 0 on the current tree (18/18 present).

**Exit criteria**
- With a deliberately broken target spec, `--strict` names the specific missing
  element. Restore the spec afterwards.
- On a clean tree, `--strict` exits 0.

---

### WP-W6 — provision the C4 render toolchain

`python scripts/generate_c4.py` with no flags hard-exits when PlantUML or Graphviz
is missing, which makes the regenerate-and-commit rule unrunnable on a clean
machine. It also matters for the gate: with a renderer present `--check`
byte-compares the SVGs, and without one it only checks that they exist.

- Fix `scripts/setup_c4_tooling.sh`: it runs `apt-get install` with no preceding
  `apt-get update`, which fails on a fresh container, and it errors out when
  `java` is absent instead of installing a JRE.
- Add `graphviz`, a headless JRE, and the pinned PlantUML jar to
  `docker/Dockerfile.dev`, exporting `PLANTUML_JAR` so `find_plantuml` resolves it.
- Add a CI step that installs the toolchain and runs `generate_c4.py --check` as a
  real drift gate.
- Settle the SVG bytes here, not in W4: PlantUML version differences change SVG
  output, so the committed SVGs may not match what the pinned version renders. If
  they differ, regenerate and commit them as part of this WP.

**Exit criteria**
- The toolchain installs from a clean container.
- `python scripts/generate_c4.py` with no flags succeeds.
- `--check` with a renderer present exits 0 on a clean tree.

---

### WP-W4 — hooks and `.claude/settings.json`

The repo has no `.claude/settings.json` today: no hooks, no permissions allowlist.

- **PreToolUse** blocking `Edit|Write` on `docs/architecture/target/**` and
  `docs/capabilities/**` — the zero-exception rule that most deserves to be a hook
  rather than a sentence.
- **PostToolUse** on `Edit|Write` matching `src/**/*.{cpp,hpp}`, `launch/**`,
  `msgs/**` → `generate_c4.py --check`, for feedback right after the edit that
  caused the drift.
- **Stop** hook, capturing both streams (`2>&1`) because the scripts disagree about
  which they print to:
  - `generate_c4.py --check` — blocking, green today
  - `check_architecture_gap.py --strict` — blocking, green today
  - `check_traceability.py` **without** `--strict` — advisory
  The omitted `--strict` must be commented in the file: it fails today on 17
  Approved-but-unverified requirements, and the work that would fix it is out of
  scope. A hook that can never pass is a hook that gets disabled. Revisit when
  those requirements get tests. Comment too that both `check_*` scripts rewrite
  their reports as a side effect, so a green turn can still leave the tree dirty.
- `permissions.allow` for `python scripts/*.py`, `bash scripts/*.sh`, and common
  read-only git — every checker run currently prompts.
- Then delete the standing-rule bullets from `CLAUDE.md` that the hooks now
  enforce, leaving only what hooks do not cover.

**Exit criteria**
- An Edit on `docs/architecture/target/CAP-001-photogrammetry.yaml` is refused.
- A turn ending with stale C4 views is blocked, and the message names the stale files.
- A clean tree ends turns normally.

---

### WP-W5 — skill hygiene and model routing

- `disable-model-invocation: true` on the `c4` and `report` skills — both write
  generated artifacts and should be deliberate (`/c4`, `/report`).
- Add `model:` frontmatter per the routing table below, using **valid aliases
  only** (`sonnet`, `opus`, `haiku`, `best`, `sonnet[1m]`, `opus[1m]`, `opusplan`).
  There is no `effort:` key — do not add one.
- Push each standing rule into the skill that owns it: regenerate-and-commit-C4
  into `c4`, requirement-UIDs-only-in-`.sdoc` into `requirements`,
  gap-check-after-merge into `capability`.
- Final `CLAUDE.md` trim; link this page for routing and habits.

**Exit criteria**
- `CLAUDE.md` well under 100 lines.
- Every rule removed from `CLAUDE.md` lives in exactly one of a skill, a hook, or
  this page — never two.

---

## Routing

Which model class handles which kind of work. Designer-class work is where a
wrong answer is expensive and hard to notice; generation work is where the output
is checked by a script immediately afterwards.

| Work | Class | Why |
|---|---|---|
| `capability` — stakeholder tasks, target architecture, gap analysis | designer (`opus`) | Sets the target everything else is measured against; errors propagate into every downstream WP. |
| `design` — feature and change design docs | designer (`opus`) | Decides interfaces and structure before code exists to correct them. |
| `test-plan` — verification planning, requirement linkage | designer (`opus`) | Deciding what counts as verified is a judgement call, not a transcription. |
| `requirements` — StrictDoc authoring | designer (`opus`) | UIDs are permanent and referenced from code markers. |
| `architecture` — subsystem and use-case docs | designer (`opus`) | Same reasoning as design. |
| `c4` — generate views from code | generation (`sonnet`) | Output is mechanical and `--check` verifies it. |
| `report` — status, traceability, verification reports | generation (`sonnet`) | Derived from checker output, not from judgement. |
| Work-package implementation | `sonnet` under the approval gate | Volume work, gated by a fresh reviewer against written exit criteria. |
| Work-package review | strongest available | The gate is only worth having if the reviewer is at least as strong as the implementer. |

---

## Session habits

Not enforceable as code — these are the practices that make the enforceable parts
work.

**One named session per work package.** Name the session after the WP. A session
carrying two WPs blends their context, and the second one inherits the first's
assumptions.

**`/clear` between work packages.** Stale context is worse than no context: it
produces confident answers built on a file that has since changed.

**The two-correction rule.** If you have corrected the same misunderstanding
twice, stop correcting and restate the task from scratch in a fresh session. A
third correction almost never lands — the wrong frame is already established, and
each correction is interpreted inside it.

**Spec first, then interview.** Before implementation, get the spec written down
and read it back. Ambiguity found by reading a spec costs a minute; ambiguity
found by reviewing a diff costs the diff.

**Verification is evidence, not assertion.** "Tests pass" is not a result. The
command, its output, and its exit code are the result. This applies to what you
accept from a subagent as much as to what you report.

**Worktrees for parallel work packages.** Two sessions in one working tree will
interleave edits and produce a diff neither of them intended. `git worktree add`
per WP if you want them running at once.

**The approval gate is not optional and not self-administered.** The implementer
does not review its own work, and the reviewer does not see how the work was done.
If you find yourself reasoning about why a finding is unfair, that is the gate
working — verify the finding against the code, not against your memory of writing it.
