# Local LLM Agent Backend

Runs an [Ollama](https://ollama.com) server on your local GPU as a drop-in
replacement for the Anthropic API.  The agent workers connect to it via the
OpenAI-compatible endpoint Ollama exposes at port `11434`.

> **Linux note:** `host.docker.internal` is not available on Linux Docker Engine.
> Use the bridge gateway IP instead (typically `172.18.0.1`):
> ```
> LLM_BASE_URL=http://172.18.0.1:11434/v1
> ```
> Confirm your gateway: `docker inspect <container> | grep Gateway`

---

## Requirements

- Docker with the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- A CUDA-capable GPU with enough VRAM for your chosen model (see table below)

---

## Recommended Models

| Model | VRAM | Notes |
|---|---|---|
| `qwen2.5-coder:14b` | ~9 GB | Best fit for a 12 GB card (RTX 3060). Strong at ROS2/C++/Python |
| `qwen2.5-coder:32b` | ~20 GB | Needs a 24 GB card (RTX 3090). Noticeably stronger reasoning |
| `deepseek-coder-v2:16b` | ~10 GB | Good alternative for the 3060 |

The default is `qwen2.5-coder:14b`.  Override with the `LLM_MODEL` env var.

---

## Quick Start

**1. Start Ollama**

```bash
cd docker/local-agent
docker compose up -d
```

The first run pulls the model (~9 GB for the default).  Weights are cached in
the `ollama-models` Docker volume and reused on subsequent starts.

**2. Start the agent workforce**

```bash
cd docker
LLM_BACKEND=openai_compat \
LLM_BASE_URL=http://host.docker.internal:11434/v1 \
LLM_API_KEY=none \
LLM_MODEL=qwen2.5-coder:14b \
docker compose up -d
```

Or add the vars to a `.env` file in `docker/` so you don't have to repeat them:

```ini
# docker/.env
LLM_BACKEND=openai_compat
LLM_BASE_URL=http://host.docker.internal:11434/v1
LLM_API_KEY=none
LLM_MODEL=qwen2.5-coder:14b
```

Then just:

```bash
docker compose up -d
```

---

## Switching Models

```bash
# Stop the workforce first
docker compose down

# Restart Ollama with a different model
cd docker/local-agent
LLM_MODEL=qwen2.5-coder:32b docker compose up -d

# Restart workers with the new model name
cd ..
LLM_MODEL=qwen2.5-coder:32b docker compose up -d
```

Previously pulled models stay cached in the volume — switching back is instant.

---

## Backend Reference

The `LLM_BACKEND` env var controls which API the agent workers call:

| `LLM_BACKEND` | Where | Extra vars needed |
|---|---|---|
| `anthropic` (default) | Anthropic API | `ANTHROPIC_API_KEY` |
| `openai_compat` | Any OpenAI-compatible endpoint | `LLM_BASE_URL`, `LLM_API_KEY`, `LLM_MODEL` |

**Moonshot K2 (API):**

```ini
LLM_BACKEND=openai_compat
LLM_BASE_URL=https://api.moonshot.cn/v1
LLM_API_KEY=sk-...
LLM_MODEL=kimi-k2
```

**Mock mode (no API calls — for workflow testing):**

```ini
AGENT_MOCK=true
```

---

## Troubleshooting

**GPU not detected**
Verify the NVIDIA Container Toolkit is installed and `docker info | grep -i nvidia` shows the runtime.

**First start is slow**
The 14B model (~9 GB) is downloaded from `ollama.com` on first start and cached in the `ollama-models` Docker volume.  Subsequent starts skip the download entirely — the entrypoint checks for an existing model before pulling.

**Workers connect before Ollama is ready**
The `entrypoint` in `docker-compose.yml` waits 5 seconds before pulling and serving.  If workers start too fast, restart them: `docker compose restart orchestrator ros2-worker sim-worker ml-worker deploy-worker`
