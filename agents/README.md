# Agent System — Setup Guide

Multi-agent orchestration for `drone_autonomy_platform` using Temporal + Claude API.

## Architecture

```
Your request
    ↓
Orchestrator (Temporal workflow)
    ↓ routes to...
┌──────────────────────────────────────────────┐
│  Domain Agents        Operations Agents      │
│  · Perception Dev     · Sim & Test           │
│  · Navigation Dev     · ML Pipeline          │
│  · Control Dev ⚠️     · Deploy               │
│  · Autonomy Dev       · Code Review          │
│  · Comms Dev          · Infra & Interfaces   │
│  · Safety Dev ⚠️                              │
└──────────────────────────────────────────────┘
    ↓ each agent calls...
Claude API (with domain-specific prompts + tools)
    ↓ tools execute against...
Your workspace (read/write files, colcon build, run tests)
```

## Prerequisites

- Docker + Docker Compose V2
- Anthropic API key ([console.anthropic.com](https://console.anthropic.com))
- (Optional) NVIDIA GPU for ML pipeline worker
- (Optional) SSH access to Orin Nano for deploy worker

## Quick Start

### Option A: Docker Compose (recommended)

```bash
# 1. From the repo root, create .env
cp .env.example .env
# Edit .env → add your ANTHROPIC_API_KEY

# 2. Start the agent system
docker compose -f docker-compose.agents.yaml up -d

# 3. Verify everything is running
docker compose -f docker-compose.agents.yaml ps

# 4. Open Temporal UI
open http://localhost:8080

# 5. Trigger your first workflow
docker compose -f docker-compose.agents.yaml exec orchestrator \
  python -m agents.cli feature "Add a basic camera node to src/perception"
```

### Option B: Local development (no Docker)

```bash
# 1. Install Temporal server locally
# Option: brew install temporal  (macOS)
# Option: Download from github.com/temporalio/cli/releases
temporal server start-dev --ui-port 8080

# 2. Install Python deps
pip install -r agents/requirements.txt

# 3. Set your API key
export ANTHROPIC_API_KEY=sk-ant-api03-...

# 4. Start workers (each in a separate terminal)
python -m agents.orchestrator.worker     # Terminal 1
python -m agents.ros2_worker.worker      # Terminal 2
python -m agents.sim_worker.worker       # Terminal 3
python -m agents.ml_worker.worker        # Terminal 4
python -m agents.deploy_worker.worker    # Terminal 5

# 5. Trigger a workflow
python -m agents.cli feature "Add obstacle detection to perception pipeline"
```

## CLI Commands

```bash
# Start a new feature workflow
python -m agents.cli feature "Add landing zone detection using depth estimation"

# Deploy to Orin Nano
python -m agents.cli deploy v0.3.0

# Retrain and deploy an ML model
python -m agents.cli model "Retrain YOLOv8 with new crop health dataset"

# Approve a waiting workflow (after human review)
python -m agents.cli approve feature-add-landing-zone-detection

# Check workflow status
python -m agents.cli status feature-add-landing-zone-detection
```

## How It Works

1. **You submit a request** via the CLI
2. **Orchestrator** analyzes your intent, identifies affected packages, classifies safety-criticality
3. **Domain agents** execute in sequence — each calls Claude with a domain-specific system prompt and tools to read/write files, build, and test
4. **Human gates** pause the workflow before simulation runs, deploys, or safety-critical merges — you approve via CLI or Temporal UI
5. **Results** are visible in Temporal UI with full execution history, retry logs, and timing

## Task Queues

| Queue | Worker | Agents |
|-------|--------|--------|
| `orchestrator` | orchestrator | Orchestrator, Code Review, Infra |
| `ros2-dev` | ros2-worker | Perception, Navigation, Control, Autonomy, Comms, Safety |
| `simulation` | sim-worker | Sim & Test |
| `ml-pipeline` | ml-worker | ML Pipeline |
| `deployment` | deploy-worker | Deploy |

## Safety-Critical Paths

Changes to `src/control/` or `src/safety/` automatically trigger:
- Extended code review (DO-178C Level D checklist)
- Mandatory human approval gate
- 90% test coverage requirement
- Extra logging and bounded-output checks

## Files Added

```
drone_autonomy_platform/
├── docker-compose.agents.yaml    # Agent infrastructure
├── .env.example                  # API key template
└── agents/
    ├── cli.py                    # CLI entry point
    ├── requirements.txt          # Python dependencies
    ├── orchestrator/
    │   ├── Dockerfile
    │   ├── worker.py             # Main entry point
    │   ├── workflows.py          # Temporal workflow definitions
    │   └── activities.py         # Activity implementations
    ├── ros2-worker/
    │   ├── Dockerfile
    │   └── worker.py
    ├── sim-worker/
    │   ├── Dockerfile
    │   └── worker.py
    ├── ml-worker/
    │   ├── Dockerfile
    │   └── worker.py
    ├── deploy-worker/
    │   ├── Dockerfile
    │   └── worker.py
    └── shared/
        ├── llm_client.py         # Claude API wrapper + tool executor
        ├── tools.py              # Tool definitions for function calling
        └── prompts.py            # All agent system prompts
```

## Extending

**Add a new agent:**
1. Add system prompt to `agents/shared/prompts.py`
2. Add tool set to `agents/shared/tools.py`
3. Add routing in `agents/orchestrator/activities.py`

**Add a new tool:**
1. Define schema in `agents/shared/tools.py`
2. Implement execution in `agents/shared/llm_client.py` → `execute_tool()`

**Add a new workflow:**
1. Define in `agents/orchestrator/workflows.py`
2. Add CLI command in `agents/cli.py`
