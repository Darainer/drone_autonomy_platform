#!/usr/bin/env bash
# Local environment setup for drone_autonomy_platform.
# Safe to re-run — checks before installing.
#
# Usage: bash scripts/setup.sh [--gpu]
#   --gpu   Also install NVIDIA Container Toolkit (required for local Ollama)

set -euo pipefail

GPU=false
for arg in "$@"; do [[ "$arg" == "--gpu" ]] && GPU=true; done

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
ok()   { echo -e "${GREEN}  ✔ $*${NC}"; }
warn() { echo -e "${YELLOW}  ⚠ $*${NC}"; }
fail() { echo -e "${RED}  ✘ $*${NC}"; exit 1; }
info() { echo -e "  → $*"; }

echo ""
echo "═══════════════════════════════════════════"
echo "  drone_autonomy_platform — environment setup"
echo "═══════════════════════════════════════════"
echo ""

# ── Docker ───────────────────────────────────────
echo "[ Docker ]"
if command -v docker &>/dev/null; then
    ok "Docker $(docker --version | awk '{print $3}' | tr -d ',')"
else
    fail "Docker not found. Install from https://docs.docker.com/engine/install/"
fi

if docker compose version &>/dev/null; then
    ok "Docker Compose V2 $(docker compose version --short)"
else
    fail "Docker Compose V2 not found. Install the compose plugin: https://docs.docker.com/compose/install/"
fi

if docker info &>/dev/null; then
    ok "Docker daemon running"
else
    fail "Docker daemon not running. Start it with: sudo systemctl start docker"
fi

# ── .env ─────────────────────────────────────────
echo ""
echo "[ Environment ]"
ENV_FILE="$(dirname "$0")/../docker/.env"
if [[ -f "$ENV_FILE" ]]; then
    ok ".env file exists"
    if grep -q "ANTHROPIC_API_KEY=sk-" "$ENV_FILE"; then
        ok "ANTHROPIC_API_KEY set"
    else
        warn "ANTHROPIC_API_KEY not set in docker/.env (required for Anthropic backend)"
    fi
else
    warn "docker/.env not found — creating from template"
    cat > "$ENV_FILE" << 'EOF'
# Required for Anthropic backend
ANTHROPIC_API_KEY=

# Uncomment for local Ollama backend
# LLM_BACKEND=openai_compat
# LLM_BASE_URL=http://host.docker.internal:11434/v1
# LLM_API_KEY=none
# LLM_MODEL=qwen2.5-coder:14b

# Deploy worker (optional)
# ORIN_HOST=192.168.1.100
# ORIN_USER=jetson
EOF
    info "Created docker/.env — add your ANTHROPIC_API_KEY"
fi

# ── NVIDIA / GPU ──────────────────────────────────
echo ""
echo "[ GPU ]"
if command -v nvidia-smi &>/dev/null; then
    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 || echo "unknown")
    ok "NVIDIA GPU detected: $GPU_NAME"
else
    warn "nvidia-smi not found — GPU unavailable (not needed for Anthropic/Moonshot backends)"
fi

if [[ "$GPU" == "true" ]]; then
    if docker info 2>/dev/null | grep -q "nvidia"; then
        ok "NVIDIA Container Toolkit already configured"
    else
        info "Installing NVIDIA Container Toolkit..."
        curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
            | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
        curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
            | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
            | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
        sudo apt-get update -qq
        sudo apt-get install -y nvidia-container-toolkit
        sudo nvidia-ctk runtime configure --runtime=docker
        sudo systemctl restart docker
        ok "NVIDIA Container Toolkit installed and Docker restarted"
    fi
else
    if ! docker info 2>/dev/null | grep -q "nvidia"; then
        info "NVIDIA Container Toolkit not configured (run with --gpu to install, required for local Ollama)"
    fi
fi

# ── Summary ───────────────────────────────────────
echo ""
echo "═══════════════════════════════════════════"
echo "  Next steps:"
echo ""
echo "  1. Build agent containers:"
echo "     cd docker && UID=\$(id -u) GID=\$(id -g) docker compose build"
echo ""
echo "  2. Start the stack:"
echo "     docker compose up -d"
echo ""
if [[ "$GPU" == "true" ]]; then
echo "  3. Start local Ollama (GPU ready):"
echo "     cd docker/local-agent && docker compose up -d"
echo ""
fi
echo "  Temporal UI: http://localhost:8080"
echo "═══════════════════════════════════════════"
echo ""
