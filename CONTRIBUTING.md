# Contributing

## Setup

```bash
# Build with your host UID/GID — prevents root-owned files in the workspace
cd docker
DOCKER_UID=$(id -u) DOCKER_GID=$(id -g) docker compose build
docker compose up -d
```

See `agents/README.md` for full agent system setup including local LLM backends.

## Development workflow

### The three nested loops

Work in this repo is engineered through three nested loops: **capability**
(stakeholder task → target architecture → gap, designer-owned) → **system**
(requirements/design/test-plan) → **implementation** (agent workforce /
Claude Code sessions). Each loop's artifacts are inputs to the next.

| Skill | Use for |
|---|---|
| `capability` | Stakeholder tasks, target architecture, gap analysis, WP handoff |
| `requirements` | Add/change requirements (StrictDoc) |
| `design` | Feature/change design docs before implementation |
| `architecture` | Subsystem/use-case architecture docs |
| `test-plan` | Verification planning + test↔requirement linkage |
| `report` | Status, traceability, verification reports |
| `c4` | Generate C4 architecture views from code |

[`CLAUDE.md`](CLAUDE.md)'s skill table is the authoritative one — it also
lists the artifact path(s) each skill owns; look there rather than here.

See [`docs/workflow/README.md`](docs/workflow/README.md) for how work is
routed to each loop and for session habits. See `docs/capabilities/` for
worked examples of a capability moving through all three loops.

### Working with Claude Code in this repo

If you use Claude Code (or the agent workforce) against this repo, here is
what you will actually run into:

- Edits to `docs/architecture/target/**` and `docs/capabilities/**` are
  **refused** by a `PreToolUse` hook — those paths are designer-owned.
  Designer sessions are launched with `DAP_DESIGNER_SESSION=1` to lift the
  block; there is no way to grant the exemption mid-session.
- C4 drift is checked in three places: right after a source edit
  (`PostToolUse`), again when the session ends (`Stop`), and again in CI
  (`c4-drift-check`).
- The session-end (`Stop`) hook also runs the gap and traceability checkers,
  but **advisory-only** — only the C4 drift check blocks. This is the
  repo's most load-bearing non-obvious decision, in short: a gate that can
  never pass is a gate that gets disabled. Today, `check_traceability.py
  --strict` fails on approved requirements that don't have a verifying test
  yet; `check_architecture_gap.py --strict` currently passes (no gaps on the
  one target spec that exists), but stays advisory because gaps are the
  *expected* state while a capability is mid-iteration, not an error. Full
  reasoning lives in `.claude/settings.json` and `docs/workflow/README.md` —
  read those before assuming either checker should be made blocking.
- The three checker scripts do not share a flag vocabulary — passing the
  wrong one exits 2:
  ```bash
  python scripts/generate_c4.py --check              # exit 1 if C4 views are stale
  python scripts/check_architecture_gap.py --strict  # exit 1 if gaps remain
  python scripts/check_traceability.py               # --strict exits 1 if an Approved requirement lacks a verifying test
  ```
- `scripts/setup_c4_tooling.sh` installs the Java/Graphviz/PlantUML toolchain
  and must be run once before `generate_c4.py` will render SVGs; without it,
  pass `--no-render`.

### The work-package approval loop

Implementation work packages (`docs/workflow/README.md`) run through
`/work-package <WP-ID>`: a fresh implementer subagent writes the change
against the WP's spec, a fresh reviewer subagent gates the diff against that
same spec's exit criteria with no knowledge of how the work was done, and
only an `APPROVE` verdict gets committed — capped at three rounds before a
human has to look. See `docs/workflow/README.md` for the full loop and the
session habits around it (naming, `/clear`, worktrees).

### What "verified" means here

Say plainly which command you ran and paste its real output and exit
code — "tests pass" is not a result, the command and its exit code are. The
honest state, so you do not invent commands that don't exist:

- The only automated test run anywhere is `pytest agents/tests/`, run by CI
  with `AGENT_MOCK=true`.
- `-DBUILD_TESTING=OFF` (every Dockerfile, every documented build) means no
  C++ test under `src/**/tests/` ever compiles.
- `ruff` is configured in `pyproject.toml` but invoked nowhere in the repo.

## Safety-Critical Code Paths

Changes to the following packages require a safety review before merge:

- `src/control/` — flight control algorithms
- `src/safety/` — failsafe and emergency handling
- `src/navigation/` — path planning

**What actually happens today:** a CI job (`safety-path-warning` in
`.github/workflows/ci.yml`) diffs each pull request against its base and, if a
changed path falls under one of the three directories above and the PR does
not carry the `safety-reviewed` label, writes an advisory warning naming the
changed files to the job summary. It always exits 0 — it does not block
anything, and requiring it via branch protection is a manual step nobody has
taken yet. Fill in the DO-178C checklist section of the pull request template
and add the `safety-reviewed` label once a human has reviewed the change.

Automatic path detection with a mandatory human approval gate before deploy is
**planned, not implemented**. Concretely: no code anywhere in this repo
inspects a diff or file list to detect these paths outside the CI job above;
the Temporal `NewFeatureWorkflow` gate (`agents/orchestrator/workflows.py`)
keys only on a `safety_critical` boolean that an LLM fills in from a prompt,
with no path detection behind it — that is true regardless of how the
workflow is launched. The gate's fate then depends on the entry point:
the documented one, `scripts/task.sh` → `scripts/submit_task.py`, hardcodes
`auto_approve=True` with no flag to turn it off, so the wait-for-human-approval
branch never executes for plans submitted that way. The workflow can also be
started directly via `python -m agents.cli feature`, which leaves
`auto_approve` at its dataclass default of `False`
(`agents/orchestrator/workflows.py`) — for a plan with `safety_critical: true`
launched that way, the gate does wait, and `agents/cli.py approve
<workflow-id>` is what releases it. Either way, the gate is not a path-based
safety check: it only ever fires when the boolean an LLM produced happens to
be `True`.

## Pull Request Process

1. Fork the repository and create a feature branch off `main`
2. Make your changes inside the Docker environment to ensure they build cleanly
3. Run the smoke test locally before pushing:
   ```bash
   docker build -t drone_autonomy_platform .
   docker run --rm drone_autonomy_platform \
     bash /ws/src/drone_autonomy_platform/scripts/smoke_test.sh
   ```
4. Submit a PR with a description of what changed and why
5. CI builds the image (`build` job in `.github/workflows/ci.yml`), but it does
   **not** invoke `scripts/smoke_test.sh` — nothing in CI runs that script
   today, so step 3 above is the only place it actually gets exercised. Run it
   locally before pushing.

Agents do not open pull requests. `agents/shared/tools.py` gives them
`git_branch`, `git_commit`, and `git_push` only — there is no PR-creation code
anywhere in `agents/`. A human still has to open the PR from the pushed
branch. Review the Temporal UI for execution history and per-step logs before
merging.

## Adding a New Node

1. Create `src/<name>/` with `CMakeLists.txt`, `package.xml`, `src/<name>_node.cpp`, and `launch/<name>.launch.py`
2. Add an `install(TARGETS <name>_node DESTINATION lib/${PROJECT_NAME})` rule in `CMakeLists.txt`
3. Add the package to `launch/platform_core.launch.py` (or `platform.launch.py` if it requires Isaac ROS)
4. Add the expected node name to the `smoke_test.sh` check list (this keeps the
   script accurate for contributors who run it locally per step 3 of the pull
   request process above — no automation runs `smoke_test.sh` today, so this
   is a local-only check, not a CI gate)

## Perception / Isaac ROS

The `perception` package requires the NVIDIA Isaac ROS apt registry and is only buildable on
Jetson hardware. Do not add Isaac ROS dependencies to any other package.
