# DES-002 — Route Fused Perception Output (`/sensor_data`) to the Mission Manager

**Status:** Draft
**Safety-critical:** no (touches `src/perception/` and `src/autonomy/` only)

## Summary

`perception_node` publishes `drone_autonomy_msgs/SensorData` on the private
topic `~/sensor_data` and nothing consumes it — the perception subsystem is a
dead end. Per the system architecture, `perception_node` is the platform's
sensor-fusion point and the mission manager (`autonomy_node`, behavior tree)
is the intended consumer of perceived information: COMP-5 explicitly budgets
"the time for the mission manager to make a decision based on the perceived
information", and PLAT-2 budgets perception output → control command. This
design establishes the canonical topic `/sensor_data` and makes
`autonomy_node` subscribe to it as its decision input, closing the
perception → decision half of the E2E-2/E2E-3 chains (DES-001 closed the
decision → control half).

## Requirements addressed

| UID | How this design satisfies it |
|---|---|
| PER-1 (Draft) | `perception_node` `~/sensor_data` remapped to canonical `/sensor_data`; already publishes per camera frame. |
| AUT-1 (Draft) | `autonomy_node` gains a `/sensor_data` subscription; latest fused sample feeds mission decisions. |
| COMP-5 | Gives the mission manager the perceived-information input its 20 ms decision budget is measured against. |
| PLAT-2 | Makes "perception output → control command" a real, measurable path: `/sensor_data` → decision → `/mission` → … → `/attitude_command`. |

## Current state

- [c4/topics.md](../architecture/c4/topics.md): `/perception_node/sensor_data`
  — **dangling (no subscriber)**; `autonomy_node` shows `1 pub / 0 sub`
  ([c4/level3_component_autonomy.md](../architecture/c4/level3_component_autonomy.md)).
- `perception_node` fuses OAK-D RGB (`/oak/rgb/image_raw`), stereo depth, and
  RF-DETR detections (`/detections`), republishing on image arrival. Today it
  populates only the `image` field of `SensorData`; `imu`, `gps`, `lidar` are
  defined in the message but not yet filled, and detections are received but
  not forwarded.
- README "Perception Pipeline" names `perception_node` as *(sensor fusion)*;
  `docs/architecture/perception_architecture.md` shows it as the terminal
  consumer of the detection/tracking pipeline — i.e., the fused world picture
  is supposed to leave perception through this one topic.

## Proposed design

Wiring first (this design), content enrichment later (future design):

1. **Canonical topic.** Remap `perception_node` `~/sensor_data` →
   `/sensor_data` in `src/perception/launch/` (same DES-001 pattern; QoS stays
   `SensorDataQoS` — latest-sample semantics suit decision making, matching
   the platform convention for camera-derived topics).
2. **Consumer.** `autonomy_node` subscribes to `/sensor_data`
   (`SensorDataQoS`, depth 1), caches the latest message, and stamps mission
   decisions from it. Marker: `// Implements: AUT-1`.
3. **Producer marker.** `// Implements: PER-1` at the `~/sensor_data`
   publisher in `perception_node.cpp`.
4. **Fusion content gap** (explicitly out of scope here): forwarding
   detections/depth/IMU inside `SensorData` instead of only the RGB frame.
   That is a message/content change (`infra` step to evolve
   `msgs/ros2/SensorData.msg` or add a `PerceivedObjects.msg`) and deserves
   its own design once mission logic defines what it needs. Raw-image-only
   payload is acknowledged as bandwidth-heavy for a 20 ms decision loop —
   flagged in Open questions.

Agent decomposition when implemented via `submit_task.py`:
`perception-dev` (launch remap + marker) → `autonomy-dev` (subscription) →
`infra` (regenerate C4 views) → `sim-test` (chain check).

## Interfaces

| Direction | Topic/Service | Type | QoS | Notes |
|---|---|---|---|---|
| perception_node → | `/sensor_data` | `drone_autonomy_msgs/SensorData` | SensorDataQoS (best-effort, depth 5) | was `~/sensor_data`, dangling |
| → autonomy_node | `/sensor_data` | `drone_autonomy_msgs/SensorData` | SensorDataQoS, depth 1 | new subscription |

## Safety impact

None direct — neither package is in the DO-178C review scope
(`src/control/`, `src/safety/`, `src/navigation/` untouched). Indirect:
mission decisions become sensor-dependent, so a stale or silent
`/sensor_data` feed becomes a failure mode; the future fusion-content design
must define staleness handling (e.g., decision veto on stale data), and the
watchdog scope in `src/safety/` should eventually cover this topic's liveness.

## Test strategy

- **Test (SITL, `sim-test` queue):** publish a synthetic `/sensor_data`
  sample; assert `autonomy_node`'s next `/mission` reflects receipt
  (`# Verifies: AUT-1` once implemented).
- **Test (smoke):** extend `scripts/smoke_test.sh` topic checks —
  `/sensor_data` must have ≥1 publisher and ≥1 subscriber.
- **Analysis:** `python scripts/generate_c4.py --check` passes with
  `/sensor_data` shown **connected** in `c4/topics.md` (PER-1).
- Test plan rows to be added to TP-001 (or a new TP-002) when this design is
  Approved and PER-1/AUT-1 move to Approved.

## Alternatives considered

- **Autonomy subscribes to `/detections` directly.** Rejected: bypasses the
  fusion point, couples mission logic to a raw detector format, and breaks
  the layered perception architecture (detector → tracker → fusion → consumer).
- **Navigation consumes `/sensor_data` instead.** Rejected for now: Nav2
  costmap integration wants obstacle/depth layers, not a fused
  camera-snapshot message; the architecture routes *decisions* through the
  mission manager (COMP-5), with navigation acting on `/mission`.
- **Delete the publisher as dead code.** Rejected: the sensor-fusion output
  is load-bearing in the documented architecture (README, perception
  architecture); the gap is the missing consumer, not the producer.

## Open questions

- Should `SensorData` carry detections/tracked objects (or should a leaner
  `PerceivedObjects.msg` replace the raw image for the decision path)?
  Full frames at 30 Hz to the BT is wasteful once real mission logic exists.
- Staleness policy: how old may the newest `/sensor_data` sample be before
  the mission manager must refuse to act on it (ties into E2E-2's 250 ms)?
