# Agent System

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
UID=$(id -u) GID=$(id -g) docker compose build

# 3. Start everything
docker compose up -d

# 4. Open Temporal UI
open http://localhost:8080
```

## Submitting Tasks

Tasks are submitted from the host via `scripts/task.sh`, which runs inside the
orchestrator container where all dependencies are installed.

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

## LLM Backends

Set via env vars in `docker/.env` or your shell:

| `LLM_BACKEND` | Required vars | Example model |
|---|---|---|
| `anthropic` (default) | `ANTHROPIC_API_KEY` | `claude-sonnet-4-6` |
| `openai_compat` | `LLM_BASE_URL`, `LLM_API_KEY`, `LLM_MODEL` | `qwen2.5-coder:14b` (Ollama), `kimi-k2` (Moonshot) |

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

## Task Queues

| Queue | Worker | Agents |
|---|---|---|
| `orchestrator` | orchestrator | infra, code-review |
| `ros2-dev` | ros2-worker | perception-dev, nav-dev, control-dev, autonomy-dev, comms-dev, safety-dev |
| `simulation` | sim-worker | sim-test |
| `ml-pipeline` | ml-worker | ml-pipeline |
| `deployment` | deploy-worker | deploy |

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

CLAUDE.md                     # Claude Code reference: plan schema, valid agents, rework loop
```

## Extending

**Add a new agent role:**
1. Add system prompt to `agents/shared/prompts.py`
2. Add tool set to `agents/shared/tools.py`
3. Add routing entry in `agents/orchestrator/activities.py` → `TOOL_SETS`
4. Add to `valid_agents` and `agent_to_queue` in `analyze_intent`
5. Update `CLAUDE.md` agent table

**Add a new tool:**
1. Define schema in `agents/shared/tools.py`
2. Implement in `agents/shared/llm_client.py` → `execute_tool()`
3. Add to relevant agent tool sets in `tools.py`
