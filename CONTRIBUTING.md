# Contributing

## Setup

```bash
# Build with your host UID/GID — prevents root-owned files in the workspace
cd docker
DOCKER_UID=$(id -u) DOCKER_GID=$(id -g) docker compose build
docker compose up -d
```

See `agents/README.md` for full agent system setup including local LLM backends.

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
