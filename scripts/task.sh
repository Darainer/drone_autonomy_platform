#!/usr/bin/env bash
# Submit a task to the agent workforce from the host.
# Runs submit_task.py inside the orchestrator container where all deps are installed.
#
# Usage:
#   scripts/task.sh "description"
#   scripts/task.sh "description" --plan '{"summary":...}'
#   scripts/task.sh "description" --plan '{"summary":...}' --rework "tests failed: ..."

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/../docker/docker-compose.yml"

exec docker compose -f "${COMPOSE_FILE}" exec orchestrator \
  python3 scripts/submit_task.py "$@"
