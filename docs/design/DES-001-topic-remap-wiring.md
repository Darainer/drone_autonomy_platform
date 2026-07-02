# DES-001 — Connect the Mission/Trajectory/Command Topic Chain via Launch Remappings

**Status:** Implemented
**Safety-critical:** yes (touches `src/control/` and `src/navigation/` launch files)

## Summary

The core data path autonomy → navigation → control → communication does not
exist at runtime: each node publishes or subscribes on private (`~/`) topics
that expand to per-node names (`/autonomy_node/mission` vs
`/navigation_node/mission`) and no launch file remaps them. This design
introduces three canonical absolute topics — `/mission`, `/trajectory`,
`/attitude_command` — and remaps each node's private name onto them in the
per-package launch files. It also extends `scripts/generate_c4.py` to parse
launch-file remappings so the generated architecture views reflect actual
runtime wiring.

## Requirements addressed

| UID | How this design satisfies it |
|---|---|
| PLAT-2 | Perception output can only become a control command within 30 ms if the decision→control→bridge topic chain is actually connected. |
| E2E-2 | Threat response (sensor→action ≤ 250 ms) requires the full mission→trajectory→attitude-command flow to reach the flight-controller bridge. |
| E2E-3 | Target-tracking course adjustments flow over the same chain. |

## Current state

`docs/architecture/c4/topics.md` (before this change) flagged three
same-basename private-topic pairs as **needs remap** and the container view
showed them as dashed intent, not connections:

- `autonomy_node` pub `/autonomy_node/mission` ↔ `navigation_node` sub `/navigation_node/mission`
- `navigation_node` pub `/navigation_node/trajectory` ↔ `control_node` sub `/control_node/trajectory`
- `control_node` pub `/control_node/attitude_command` ↔ `communication_node` sub `/communication_node/attitude_command`

## Proposed design

Keep the node code unchanged (private `~/` names are the ROS2-idiomatic
default and stay remappable per deployment). Establish canonical absolute
topic names and remap both ends onto them in each package's launch file:

| Canonical topic | Publisher (remap) | Subscriber (remap) |
|---|---|---|
| `/mission` | `autonomy_node` `~/mission` | `navigation_node` `~/mission` |
| `/trajectory` | `navigation_node` `~/trajectory` | `control_node` `~/trajectory` |
| `/attitude_command` | `control_node` `~/attitude_command` | `communication_node` `~/attitude_command` |

Tooling change: `scripts/generate_c4.py` gains a launch-file pass that reads
`remappings=[(...)]` from `launch_ros` `Node(...)` actions (matched by node
name) and applies them before topic matching, so remapped chains render as
connected edges and stale *needs remap* warnings disappear.

## Interfaces

| Direction | Topic/Service | Type | QoS | Notes |
|---|---|---|---|---|
| autonomy_node → | `/mission` | `drone_autonomy_msgs/Mission` | depth 10, reliable | was `~/mission` |
| → navigation_node | `/mission` | `drone_autonomy_msgs/Mission` | depth 10, reliable | was `~/mission` |
| navigation_node → | `/trajectory` | `drone_autonomy_msgs/Trajectory` | depth 10, reliable | was `~/trajectory` |
| → control_node | `/trajectory` | `drone_autonomy_msgs/Trajectory` | depth 10, reliable | was `~/trajectory` |
| control_node → | `/attitude_command` | `drone_autonomy_msgs/AttitudeCommand` | depth 10, reliable | was `~/attitude_command` |
| → communication_node | `/attitude_command` | `drone_autonomy_msgs/AttitudeCommand` | depth 10, reliable | was `~/attitude_command` |

Out of scope: `/perception_node/sensor_data` has no consumer anywhere in the
repo — that is missing functionality (a future subscriber in
autonomy/navigation), not a wiring defect; it stays listed as dangling.
`/visualization/detection_overlay` is consumed externally by
`rqt_image_view` and is intentional.

## Safety impact

Remapping only renames topics; no behavioral logic changes. Failure modes
considered: a typo in a remap silently disconnects the chain again — mitigated
by the `generate_c4.py --check` drift gate plus the topic inventory, which
now derives connectivity from launch files and would show the break. The
battery failsafe path (SAF-1/SAF-2) uses absolute `/mavros/*` names and is
unaffected.

## Test strategy

- **Test (smoke):** `scripts/smoke_test.sh` already verifies the nodes launch;
  extend later with `ros2 topic info /mission --verbose`-style pub/sub count
  checks (tracked in TP-001 environments).
- **Analysis (this change):** `python scripts/generate_c4.py --check` passes
  and `docs/architecture/c4/topics.md` shows `/mission`, `/trajectory`,
  `/attitude_command` as **connected** with zero remap warnings.
- SITL end-to-end latency verification remains per
  [TP-001](../test_plans/TP-001-latency-and-safety.md).

## Alternatives considered

- **Hardcode absolute topic names in the node source.** Rejected: loses
  per-deployment remappability (e.g., namespaced multi-drone swarm launch),
  and touches safety-critical C++ for what is a composition concern.
- **Remap only the subscriber side onto the publisher's expanded name**
  (e.g., navigation subscribes `/autonomy_node/mission`). Rejected: couples
  subscribers to publisher node names; canonical names keep the contract
  node-agnostic.

## Open questions

- Should `/mission`, `/trajectory`, `/attitude_command` move under a
  namespace (e.g. `/dap/...`) once multi-vehicle launches exist?
