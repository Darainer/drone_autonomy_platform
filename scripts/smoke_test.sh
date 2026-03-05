#!/bin/bash
set -e

# ROS2 + workspace are sourced by the container entrypoint (/ros_entrypoint.sh)

echo "=== Starting core platform ==="
ros2 launch /ws/src/drone_autonomy_platform/launch/platform_core.launch.py &
LAUNCH_PID=$!

cleanup() {
    echo "=== Shutting down ==="
    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

# Give nodes time to start
sleep 5

echo "=== Checking nodes ==="
NODES=$(ros2 node list 2>/dev/null || true)
echo "$NODES"

FAILED=0
for node in autonomy_node safety_node navigation_node control_node communication_node; do
    if echo "$NODES" | grep -q "/$node"; then
        echo "PASS: /$node"
    else
        echo "FAIL: /$node not found"
        FAILED=1
    fi
done

exit $FAILED
