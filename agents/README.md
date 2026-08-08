# Agent System

> **Status (as of 2026-08): untested prototype, not part of the working
> development loop.** This Temporal-based multi-agent workforce is real code
> that runs, but no feature has been implemented end-to-end through it and it
> is not what Claude Code sessions use day to day. Treat everything below as
> describing a prototype to evaluate, not a system to depend on.

Multi-agent orchestration for `drone_autonomy_platform` using Temporal + Claude API (or a local LLM via Ollama).

## Architecture

```
You describe a task to Claude Code
    ↓  Claude Code forms a plan and calls scripts/task.sh
Temporal Workflow (NewFeatureWorkflow)
    ↓  routes steps to task queues
┌──────────────────────────────────────────────┐
│  Domain Agents          Operations Agents    │
│  · perception-dev       · sim-test           │
│  · nav-dev              · ml-pipeline        │
│  · control-dev  ⚠️      · deploy             │
│  · autonomy-dev         · code-review        │
│  · comms-dev            · infra              │
│  · safety-dev   ⚠️                           │
└──────────────────────────────────────────────┘
    ↓  each agent calls...
LLM backend (Anthropic API  or  local Ollama)
    ↓  tools execute against...
/workspace (read/write files, colcon build, run tests, git)
```

⚠️ = safety-critical path (triggers DO-178C review + human approval gate)

## My Role

*(How Claude Code is meant to drive this prototype. Moved here from CLAUDE.md,
with the entry point corrected to `scripts/task.sh`.)*

I am the orchestrator. When the user describes a task:
1. Analyze the request and form a plan (JSON, see Plan Schema below)
2. Submit it to the agent workforce via `scripts/task.sh`
3. Review the results — if tests fail or the goal is not met, revise the plan and resubmit with `--rework`

The domain agents (running on a local LLM or the Anthropic API) do the actual file editing, building, and testing.

## Prerequisites

- Docker + Docker Compose V2
- One of:
  - Anthropic API key ([console.anthropic.com](https://console.anthropic.com))
  - Local Ollama instance (see `docker/local-agent/README.md`)

## Quick Start

```bash
# 1. Set your API key
echo "ANTHROPIC_API_KEY=sk-ant-..." >> docker/.env

# 2. Build with your host UID/GID so file ownership is correct
cd docker
DOCKER_UID=$(id -u) DOCKER_GID=$(id -g) docker compose build

# 3. Start everything
docker compose up -d

# 4. Open Temporal UI
open http://localhost:8080
```

## Submitting Tasks

Tasks are submitted from the host via `scripts/task.sh`, which runs inside
the orchestrator container where all dependencies are installed. This is the
entry point — `agents/orchestrator`'s `scripts/submit_task.py` is what
`task.sh` invokes via `docker compose exec`; it is not meant to be called
directly from the host.

```bash
# Let Claude Code form the plan and call this:
scripts/task.sh "Add a GPS health monitor node to the safety package"

# With a pre-formed plan (skips the LLM analyze_intent call):
scripts/task.sh "description" --plan '{"summary":"...","steps":[...]}'

# Rework after a failed attempt:
scripts/task.sh "description" --plan '...' --rework "tests failed: missing include"
```

Claude Code acts as the orchestrator — it analyzes your request, forms a plan,
calls `task.sh`, reviews results, and resubmits with `--rework` if needed.
`task.sh` exits `0` on success, `1` if code review failed — use this to
decide whether to rework.

## Plan Schema

```json
{
  "summary": "one-line description of what will be done",
  "safety_critical": false,
  "affected_packages": ["src/perception"],
  "steps": [
    {
      "agent": "perception-dev",
      "task_queue": "ros2-dev",
      "action": "concise description of what this agent should do",
      "depends_on": []
    }
  ]
}
```

Steps execute in order. `depends_on` is informational only (not enforced by Temporal yet).

## Agents and Task Queues

| Agent | Queue | Worker | Use for |
|---|---|---|---|
| `perception-dev` | `ros2-dev` | ros2-worker | Camera, depth, detection, Isaac ROS nodes |
| `nav-dev` | `ros2-dev` | ros2-worker | Path planning, costmaps, Nav2 config |
| `control-dev` | `ros2-dev` | ros2-worker | PX4 bridge, attitude/position controllers |
| `autonomy-dev` | `ros2-dev` | ros2-worker | Mission logic, state machines, BT |
| `comms-dev` | `ros2-dev` | ros2-worker | MAVLink, telemetry, GCS interface |
| `safety-dev` | `ros2-dev` | ros2-worker | Geofence, failsafe, watchdog nodes |
| `infra` | `orchestrator` | orchestrator | CMakeLists, launch files, READMEs, msgs/ |
| `code-review` | `orchestrator` | orchestrator | Review + lint only, no edits |
| `sim-test` | `simulation` | sim-worker | SITL scenarios, unit tests |
| `ml-pipeline` | `ml-pipeline` | ml-worker | Model training, export, TensorRT |
| `deploy` | `deployment` | deploy-worker | `DeployWorkflow` only — manual trigger, separate from feature plans |

**Rules:**
- Docs, launch files, CMakeLists, READMEs → always `infra` on `orchestrator`
- `src/control/` or `src/safety/` changes → set `safety_critical: true`
- Do NOT use `deploy` as an agent in feature plans — deployment is a separate workflow triggered manually
- New message types → add an `infra` step first to define the `.msg` file

## LLM Backend

Set via env vars in `docker/.env` or your shell:

| Var | Backend | Default | Notes |
|---|---|---|---|
| `LLM_BACKEND` | both | `anthropic` | `anthropic` or `openai_compat` |
| `ANTHROPIC_API_KEY` | `anthropic` | — | required when `LLM_BACKEND=anthropic` |
| *(model)* | `anthropic` | `claude-sonnet-4-6` | hardcoded in `agents/shared/llm_client.py`, not env-configurable |
| `LLM_BASE_URL` | `openai_compat` | — | e.g. `http://localhost:11434/v1` (Ollama) |
| `LLM_API_KEY` | `openai_compat` | — | e.g. `none` (Ollama), real key (Moonshot) |
| `LLM_MODEL` | `openai_compat` | `kimi-k2` | e.g. `qwen2.5-coder:14b` (Ollama), `kimi-k2` (Moonshot) |
| `AGENT_MOCK` | both | `false` | `true` skips all LLM calls |

**Local Ollama:**
```bash
# Start Ollama first
cd docker/local-agent && docker compose up -d

# Then start agents pointing at it
cd docker
LLM_BACKEND=openai_compat \
LLM_BASE_URL=http://localhost:11434/v1 \
LLM_API_KEY=none \
LLM_MODEL=qwen2.5-coder:14b \
docker compose up -d
```

**Mock mode** (no LLM calls — for testing Temporal routing):
```bash
AGENT_MOCK=true docker compose up -d
```

## Rework Loop

After `task.sh` returns results, check:
- `result.review.passed` — did code review pass?
- `result.sim_results.result` — did tests pass?
- `result.results[*].result` — what did each agent actually do?

If any step failed, call `task.sh` again with `--rework "specific feedback"`.
Keep rework focused — identify the exact file/function that failed rather than re-running the full plan.

## File Ownership

The agent containers run as a non-root user matching your host `UID:GID` (baked
in at build time via `--build-arg UID/GID`). All files written to `/workspace`
are owned by you.

If you ever see root-owned files from a previous build:
```bash
sudo chown -R $(id -u):$(id -g) .
```

## Repository Layout

```
agents/
├── requirements.txt          # All Python deps (temporalio, anthropic, openai)
├── orchestrator/
│   ├── worker.py             # Registers workflows + activities on "orchestrator" queue
│   ├── workflows.py          # NewFeatureWorkflow, DeployWorkflow, ModelUpdateWorkflow
│   └── activities.py         # create_feature_branch, analyze_intent, run_domain_agent, ...
├── ros2_worker/worker.py     # Listens on "ros2-dev" queue
├── sim_worker/worker.py      # Listens on "simulation" queue
├── ml_worker/worker.py       # Listens on "ml-pipeline" queue
├── deploy_worker/worker.py   # Listens on "deployment" queue
└── shared/
    ├── llm_client.py         # call_agent() — Anthropic + OpenAI-compat backends, tool loop
    ├── tools.py              # Tool definitions (read_file, write_file, colcon_build, git_*, ...)
    └── prompts.py            # System prompts for each agent role

docker/
├── Dockerfile.agents         # Single image for all workers (non-root user)
├── docker-compose.yml        # All services: Temporal, DB, UI, 5 workers
└── local-agent/              # Ollama stack for local GPU inference
    ├── Dockerfile
    ├── docker-compose.yml
    └── README.md

scripts/
├── task.sh                   # Host-side task submission (runs inside container)
└── submit_task.py            # Called by task.sh via docker compose exec

CLAUDE.md                     # Claude Code session guide: build/test commands, boundaries, verification
```

## Extending

**Add a new agent role:**
1. Add system prompt to `agents/shared/prompts.py`
2. Add tool set to `agents/shared/tools.py`
3. Add routing entry in `agents/orchestrator/activities.py` → `TOOL_SETS`
4. Add to `valid_agents` and `agent_to_queue` in `analyze_intent`
5. Update the agent table above (Agents and Task Queues)

**Add a new tool:**
1. Define schema in `agents/shared/tools.py`
2. Implement in `agents/shared/llm_client.py` → `execute_tool()`
3. Add to relevant agent tool sets in `tools.py`
