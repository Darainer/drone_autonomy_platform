# DES-006 — Flight-Controller Command Bridge (Waypoint Route Following)

**Status:** Draft
**Safety-critical:** yes (`src/control`, `src/communication`; commands the flight controller)

## Summary

Today no flight command ever reaches the autopilot: `control_node` converts
`/trajectory` into an `AttitudeCommand` that `communication_node` merely logs
(`src/communication/src/communication_node.cpp` has no publishers). This design
closes that gap with a **waypoint-based bridge**: the planned route (already a
`nav_msgs/Path`) is uploaded to PX4 over the MAVROS mission interface and PX4
flies it under its own position controller (AUTO.MISSION). Waypoints are the
right primary interface because they are not time-sensitive — a brief
companion-computer stall cannot crash the vehicle mid-route. Fast-reaction
paths (obstacle avoidance, E2E-2) are **out of scope here by decision**: the
candidate mechanisms are outlined below for a dedicated follow-up study task,
to be tested and, if needed, allowed to override the normal flight limits.

## Requirements addressed

| UID | How this design satisfies it |
|---|---|
| MAP-1 / MAP-6 | The survey trajectory/mission chain currently dead-ends at a logging stub; the bridge makes dispatched routes actually fly, enabling WP-4 (TS-12 SITL e2e, TS-13 field demo) |
| STK-1 | Field demonstration requires real route execution |
| PLAT-2 / COMP-6 | For the waypoint path these budgets apply at route hand-off (upload happens before/при dispatch, not in a reactive loop); the reactive-path budgets bind the follow-up study |
| E2E-2 (⚠) / E2E-3 | **Enabled, not satisfied**: E2E-2 (obstacle avoidance ≤ 250 ms) is Approved + safety-critical and currently unverifiable because no action path exists. The follow-up fast-reaction task (below) selects and verifies the mechanism; TP-001 rows remain planned until then |
| SAF-1 | Mode-arbitration rule (D5) guarantees the bridge never fights the battery-RTL failsafe or the operator |

## Current state

- `control_node`: subscribes `/trajectory` (`drone_autonomy_msgs/Trajectory` =
  `std_msgs/Header` + `nav_msgs/Path`), publishes `/attitude_command`
  (`AttitudeCommand` roll/pitch/yaw/thrust) — a stub proportional mapping.
- `communication_node`: subscribes `/attitude_command`, `/mavros/state`;
  **no publishers** — commands terminate in a log line.
- MAVROS runs against PX4 on the Pixhawk 6X over Telem 2 (UART); the recorder
  and safety subsystems already consume `/mavros/*` topics; `battery_monitor`
  already commands RTL via `/mavros/set_mode`.
- C4: [level2_container.md](../architecture/c4/level2_container.md),
  [topics.md](../architecture/c4/topics.md) (edge
  `control_node → communication_node : /attitude_command`).

## Design decisions (fixed)

| # | Decision | Choice | Rejected alternative |
|---|---|---|---|
| D1 | Primary FC interface | **Waypoint route following**: `/trajectory` (`nav_msgs/Path`) → MAVLink mission items; PX4 position controller flies the route | Streaming attitude/rate setpoints as primary — time-critical (≥ 50 Hz keepalive), a companion stall becomes a flight-safety event; wrong default for survey missions |
| D2 | Transport & mode | **MAVROS mission protocol** (`/mavros/mission/push`, `clear`, `set_current`) + `AUTO.MISSION` via `/mavros/set_mode`; PX4-native failsafes stay authoritative | Offboard position-setpoint streaming — needs a ≥ 2 Hz keepalive and offboard-loss failsafe handling for no v1 benefit; reserved as a candidate for the fast-reaction follow-up |
| D3 | Bridge ownership | **`src/control`** (the package charter is "Trajectory tracking, MAVROS bridge to Pixhawk"): `control_node` becomes the bridge — consumes `/trajectory`, converts Path poses to mission items (local ENU → global via MAVROS conventions), uploads, arms the mode transition, supervises progress | Putting the bridge in `src/communication` — that package remains the GCS/telemetry interface; mixing FC command authority into it blurs the safety boundary |
| D4 | `AttitudeCommand` path | **Dormant, not deleted**: `control_node` stops publishing `/attitude_command`; `communication_node` drops the subscription; the message definition stays in `msgs/ros2/` reserved for the fast-reaction option O3 | Deleting the message — would churn the interface again if O3 is selected |
| D5 | Mode arbitration | Bridge may command `AUTO.MISSION` / `AUTO.LOITER` (hold) only, and only from a known-good `/mavros/state`; precedence **operator/RC > PX4 failsafe > safety_node (SAF-1 RTL) > bridge**; on any external mode change the bridge stands down (no re-assertion / mode-fighting — same single-shot philosophy as SAF-1) | Bridge re-asserting mission mode — fights the operator and the failsafe stack |
| D6 | Progress & completion | New `RouteProgress.msg` (`current_seq`, `total`, `reached_final`) published by the bridge on `/route_progress` from `/mavros/mission/reached` + `waypoints`; `autonomy_node` consumes it to drive the DES-003 `/mission_status` lifecycle (`complete`) — which is also what disarms the DES-004 recorder | Autonomy subscribing to `/mavros/mission/*` directly — couples mission logic to MAVROS types |

## Proposed design

```
/trajectory (nav_msgs/Path)
     │
     ▼
control_node  ──(bridge)──▶ /mavros/mission/push · set_current · /mavros/set_mode(AUTO.MISSION)
     │                         ▲ supervises: /mavros/state, /mavros/mission/reached, waypoints
     ▼
/route_progress (RouteProgress) ──▶ autonomy_node ──▶ /mission_status {complete}
```

Flow: on a new `/trajectory` for an active mission → validate (frame, waypoint
count, altitude bounds) → clear + push mission items → verify upload readback →
request `AUTO.MISSION` (only if armed state/mode preconditions hold per D5) →
supervise progress → publish `/route_progress`; `reached_final` triggers
autonomy's `complete`, closing the loop with the survey recorder (DES-004 D6).
Aborts (operator mode change, failsafe, upload failure) → publish progress with
an abort reason; autonomy emits `aborted`.

Implementation decomposition (agent workforce, `safety_critical: true`):
`infra` (RouteProgress.msg + remaps) → `control-dev` (bridge in control_node) →
`comms-dev` (drop `/attitude_command` subscription) → `autonomy-dev` (consume
`/route_progress` → lifecycle) → SITL scenario in the plan summary.

## Interfaces

| Direction | Topic/Service | Type | QoS | Notes |
|---|---|---|---|---|
| in (control) | `/trajectory` | `drone_autonomy_msgs/Trajectory` | reliable, depth 10 | unchanged |
| out (control) | `/mavros/mission/push` etc. | `mavros_msgs/srv/WaypointPush`, `WaypointClear`, `SetMode` | service | mission upload + mode |
| in (control) | `/mavros/state`, `/mavros/mission/reached`, `/mavros/mission/waypoints` | mavros | per mavros defaults | supervision |
| out (control) | `/route_progress` | `drone_autonomy_msgs/RouteProgress` (new) | reliable, depth 10 | D6; consumed by autonomy |
| removed | `/attitude_command` edge | — | — | D4 — msg retained, topic dormant |

## Fast-reaction options — follow-up task (outlined, deliberately NOT decided)

E2E-2 (obstacle detected → avoidance initiated ≤ 250 ms, safety-critical)
cannot be met by the waypoint path. Candidate mechanisms for the follow-up
study, to be characterized experimentally (SITL first, then HIL bench):

| Option | Mechanism | Reaction latency character | Complexity / risk |
|---|---|---|---|
| O1 | **Brake/Hold**: mode switch to `AUTO.LOITER`/brake on detection | One mode-change round-trip over Telem 2 — likely well inside 250 ms; stops, does not dodge | Lowest; likely v1 candidate for "initiation of an avoidance action" |
| O2 | **Offboard velocity override**: companion enters Offboard, streams an evasive velocity vector, then resumes the mission | Bounded by stream startup + PX4 offboard switch; true evasive motion | Medium: keepalive stream + offboard-loss failsafe handling + mission resume logic |
| O3 | **Attitude/rate override** (reuses the dormant `AttitudeCommand`) | Fastest control authority (≥ 50–100 Hz loop) | Highest: companion briefly *is* the controller; last resort |
| O4 | **PX4-native avoidance inputs** (`OBSTACLE_DISTANCE` from `/detections`-derived ranging) | Reaction handled inside PX4 | Ecosystem/maintenance caveats; offloads the safety argument to PX4 |

The study must also test **overriding normal limits** for emergency maneuvers
only — e.g. a switched parameter profile raising `MPC_XY_VEL_MAX` /
`MPC_ACC_HOR` / rate limits during an avoidance action — and measure the real
detection→motion-change latency per option against the E2E-2 budget
(TP-001). Deliverables of the follow-up: experiment report → DES-007 fixing
the mechanism → E2E-2 TP-001 test rows implemented. Until then E2E-2 remains
**planned** in the matrix.

## Safety impact

The bridge is the first component that commands flight. Mitigations: D5
precedence + stand-down (never fights operator/failsafe/SAF-1 RTL); upload
verified by readback before any mode change; no reactive authority in v1
(waypoints only); PX4 failsafes (RC loss, geofence, battery) remain fully
authoritative and are unmodified. `src/control`/`src/communication` changes ⇒
plans marked `safety_critical: true`; DO-178C review scope per
docs/standards. Failure modes: upload failure → no mode change, abort
reported; companion crash mid-flight → PX4 continues/completes the mission or
runs its own failsafe (this is the core argument for D1).

## Test strategy

SITL (PX4 SITL + MAVROS): upload/readback correctness for REF-RECT routes
(reuses WP-1 fixtures); mode-transition preconditions; progress/completion →
`/mission_status`/recorder disarm; abort paths (operator mode change mid-route,
upload failure); SAF-1 precedence (battery RTL mid-mission wins, bridge stands
down). HIL bench: Pixhawk 6X over Telem 2, mission upload + mode change with
props off. Field: first waypoint flight under RC supervision. A `test-plan`
skill pass turns this into TP rows after design approval (E2E-2 rows arrive
with the follow-up study, not this design).

## Alternatives considered

See decisions table. Additionally rejected: implementing the full reactive
stack now (couples the flyable-at-all milestone to the hardest safety
problem); GCS-mediated mission upload (QGroundControl uploads missions today
as a manual workaround, but STK-1(d)-style unattended operation and the
mission→recorder lifecycle need the onboard path).

## Open questions

- Fast-reaction mechanism (O1–O4) — deferred to the follow-up study task by
  explicit direction; not blocking this design.
- Local-ENU→global conversion source for mission items (MAVROS
  `global_position` origin vs. explicit datum in `Mission`): resolve during
  implementation against MAVROS conventions; does not change the interface.
