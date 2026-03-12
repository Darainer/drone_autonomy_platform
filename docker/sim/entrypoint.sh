#!/bin/bash
# docker/sim/entrypoint.sh
#
# Runs at container startup. Installs Pegasus Simulator into Isaac Sim's
# bundled Python (requires the /isaac-sim bind mount to be present), sources
# the ROS2 workspace, and hands off to CMD.

set -e

# ── Pegasus Simulator Python install (idempotent) ────────────────────────────
if [ -d "${ISAACSIM_PATH}" ] && [ -d "/opt/PegasusSimulator" ]; then
    echo "[entrypoint] Installing Pegasus Simulator into Isaac Sim Python..."
    ${ISAACSIM_PYTHON} -m pip install --editable \
        /opt/PegasusSimulator/extensions/pegasus.simulator --quiet || true
else
    echo "[entrypoint] WARNING: Isaac Sim not found at ${ISAACSIM_PATH}." \
         "Full SITL scenarios will not work. Mock FCU tests are unaffected."
fi

# ── Source ROS2 and workspace ─────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
if [ -f "${ROS_WS}/install/setup.bash" ]; then
    source "${ROS_WS}/install/setup.bash"
fi

exec "$@"
