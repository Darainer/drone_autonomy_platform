# Drone Autonomy Platform — Claude Code Reference

Session guide for Claude Code working directly in this repo. The Temporal
multi-agent workforce prototype under `agents/` is a separate, untested
system as of 2026-08 — see `agents/README.md`. Don't route work through it
unless a task explicitly asks you to exercise that prototype.

## Build

```bash
# x86, in-repo Docker build (all packages except perception Jetson runtime deps)
docker build -t drone_autonomy_platform .

# Jetson Orin dev container (Isaac ROS)
docker build -f docker/Dockerfile.dev -t drone_dev .

# Inside either container: colcon build as documented in README.md
cd /ws && colcon build --merge-install \
  --base-paths src/drone_autonomy_platform/msgs src/drone_autonomy_platform/src \
  --packages-ignore common \
  --cmake-args -DBUILD_TESTING=OFF
```

## Test and lint reality — do not invent commands beyond these

- The only automated test run anywhere is `pytest agents/tests/`, run by CI
  (`.github/workflows/ci.yml`) with `AGENT_MOCK=true`.
- `-DBUILD_TESTING=OFF` — in the build above and in every Dockerfile
  (`Dockerfile`, `docker/Dockerfile.dev`, `docker/Dockerfile.orin`) — means no
  C++ test under `src/**/tests/` ever compiles.
- `pyproject.toml` pins `testpaths = ["agents/tests"]`, so pytest never
  collects `tools/photogrammetry/tests/` or `src/mapping/tests/*.py`.
- `ruff` is configured in `pyproject.toml` but invoked nowhere in the repo.
- "I ran the tests" almost always means nothing was verified here. Say
  plainly which of the above you ran, and paste the real output.

## Three Nested Loops

**capability** (stakeholder task → target architecture → gap, designer-owned)
→ **system** (requirements/design/test-plan) → **implementation** (agent
workforce / Claude Code sessions).

| Skill | Use for | Key artifact |
|---|---|---|
| `capability` | Stakeholder tasks, target architecture, gap analysis, WP handoff | `docs/capabilities/CAP-*.md`, `docs/architecture/target/*.yaml` |
| `requirements` | Add/change requirements (StrictDoc) | `docs/requirements/*.sdoc` |
| `design` | Feature/change design docs before implementation | `docs/design/DES-*.md` |
| `architecture` | Subsystem/use-case architecture docs | `docs/architecture/*.md` |
| `test-plan` | Verification planning + test↔requirement linkage | `docs/test_plans/TP-*.md`, `Verifies:` markers |
| `report` | Status, traceability, verification reports | `docs/reports/*.md` |
| `c4` | Generate C4 architecture views from code | `docs/architecture/c4/*.md` |

**Standing rules:**
- After any change to nodes/topics/services: `python scripts/generate_c4.py`
  and commit the regenerated views (`--check` = drift gate).
- After any change to requirements, test plans, or `Implements:`/`Verifies:`
  markers: `python scripts/check_traceability.py` and commit the matrix.
- After merging capability work packages (or editing a target spec):
  `python scripts/check_architecture_gap.py` and commit the gap reports.
- Requirement UIDs are defined only in `docs/requirements/*.sdoc`.
- Implementation sessions never edit `docs/architecture/target/*.yaml` or
  capability docs — target changes go back to the designer (`capability` skill).

## Boundaries and verification

- Never edit `docs/architecture/target/**` or `docs/capabilities/**` — designer-owned.
- Verify by evidence, not assertion: run the real command, paste its real
  output and exit code — see "Test and lint reality" above.

## Workspace Structure

```
src/
  autonomy/        navigation/      safety/
  communication/   perception/
  control/         mapping/
msgs/              — custom ROS2 message definitions
launch/            — top-level launch files
docker/            — dev + agent containers
  local-agent/     — Ollama local LLM stack
scripts/           — generate_c4.py, check_traceability.py, check_architecture_gap.py, task.sh
docs/              — architecture, requirements (.sdoc), design, test_plans, reports
```

## Adding a New Node

1. Confirm it belongs in the target architecture (`capability` skill) if it's
   a new mission-level capability.
2. Write/update a design doc (`design` skill) before writing code.
3. Add requirements if verifiable (`requirements` skill); new `.msg` types go
   in `msgs/` first.
4. Implement under `src/<package>/`, wiring into that package's
   `CMakeLists.txt` and the relevant `launch/` file.
5. Run `generate_c4.py` / `check_traceability.py` as needed (Standing rules
   above) and commit the regenerated artifacts.
6. Build it with the commands above and show the real output — see "Test and
   lint reality" for what verification honestly means right now.
