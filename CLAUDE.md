# Drone Autonomy Platform — Claude Code Reference

## My Role

I am the orchestrator. When the user describes a task:
1. Analyze the request and form a plan (JSON, see schema below)
2. Submit it to the agent workforce via `scripts/submit_task.py`
3. Review the results — if tests fail or the goal is not met, revise the plan and resubmit with `--rework`

The domain agents (running on a local LLM) do the actual file editing, building, and testing.

---

## Submitting a Task

```bash
# With a pre-formed plan (preferred — skips the LLM analyze_intent call)
python scripts/submit_task.py "description" --plan '{"summary":...}'

# Let the agent LLM form its own plan
python scripts/submit_task.py "description"

# Rework after a failed attempt
python scripts/submit_task.py "description" --plan '{"summary":...}' --rework "tests failed: missing include in perception_node.cpp"
```

Exits `0` on success, `1` if code review failed — use this to decide whether to rework.

---

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

---

## Valid Agents and Task Queues

| Agent | Queue | Use for |
|---|---|---|
| `perception-dev` | `ros2-dev` | Camera, depth, detection, Isaac ROS nodes |
| `nav-dev` | `ros2-dev` | Path planning, costmaps, Nav2 config |
| `control-dev` | `ros2-dev` | PX4 bridge, attitude/position controllers |
| `autonomy-dev` | `ros2-dev` | Mission logic, state machines, BT |
| `comms-dev` | `ros2-dev` | MAVLink, telemetry, GCS interface |
| `safety-dev` | `ros2-dev` | Geofence, failsafe, watchdog nodes |
| `infra` | `orchestrator` | CMakeLists, launch files, READMEs, msgs/ |
| `code-review` | `orchestrator` | Review + lint only, no edits |
| `sim-test` | `simulation` | SITL scenarios, unit tests |
| `ml-pipeline` | `ml-pipeline` | Model training, export, TensorRT |
| `deploy` | `deployment` | Cross-compile, push to Orin |

**Rules:**
- Docs, launch files, CMakeLists, READMEs → always `infra` on `orchestrator`
- `src/control/` or `src/safety/` changes → set `safety_critical: true`
- New message types → add an `infra` step first to define the `.msg` file

---

## Workspace Structure

```
src/
  autonomy/        navigation/      safety/
  communication/   perception/
  control/
msgs/              — custom ROS2 message definitions
launch/            — top-level launch files
docker/            — dev + agent containers
  local-agent/     — Ollama local LLM stack
scripts/           — submit_task.py and utilities
docs/              — architecture docs
```

---

## LLM Backend

Current backend is set via env vars in `docker/.env` or shell:

| Var | Default | Options |
|---|---|---|
| `LLM_BACKEND` | `anthropic` | `anthropic`, `openai_compat` |
| `LLM_BASE_URL` | — | `http://localhost:11434/v1` (Ollama) |
| `LLM_MODEL` | — | `qwen2.5-coder:14b`, `kimi-k2` |
| `AGENT_MOCK` | `false` | `true` to skip all LLM calls |

For local Ollama: start `docker/local-agent/` first, then `docker/`.

---

## Rework Loop

After `submit_task.py` returns results, check:
- `result.review.passed` — did code review pass?
- `result.sim_results.result` — did tests pass?
- `result.results[*].result` — what did each agent actually do?

If any step failed, call `submit_task.py` again with `--rework "specific feedback"`.
Keep rework focused — identify the exact file/function that failed rather than re-running the full plan.
