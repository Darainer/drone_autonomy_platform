# DES-003 — Survey Mission Type & Coverage Trajectory Generation

**Status:** Draft (approved together with the CAP-001 implementation plan at the WP-level human gate)
**Safety-critical:** yes — touches `src/navigation`
**Capability:** CAP-001 · **Work package:** WP-1
**Test specs:** TP-002 TS-01..TS-04

## Summary

Adds a `survey` mission type to the platform: the operator supplies a polygon,
altitude, and overlap parameters; `autonomy_node` validates and dispatches the
mission; `navigation_node` generates a boustrophedon (lawnmower) coverage
trajectory whose lane and capture spacing guarantee the requested photo
overlap, publishing it into the existing DES-001 `/trajectory` → control
chain. All design decisions below are fixed — implementing agents do not
re-open them.

## Requirements addressed

| UID | How this design satisfies it |
|---|---|
| MAP-6 | Survey mission spec fields in `Mission.msg`; validation + dispatch in `autonomy_node`; mission lifecycle on `/mission_status` |
| MAP-1 | Boustrophedon generator in `navigation_node` with overlap-derived lane/capture spacing |

## Current state

- `autonomy_node` ([src/autonomy/src/autonomy_node.cpp](../../src/autonomy/src/autonomy_node.cpp)) publishes a single hardcoded `waypoints` mission on `~/mission`; no subscriptions, no `~/mission_status` publisher yet (README shows one — this design adds it).
- `navigation_node` ([src/navigation/src/navigation_node.cpp](../../src/navigation/src/navigation_node.cpp)) subscribes `~/mission`, logs it, and never publishes `~/trajectory`.
- `/mission` and `/trajectory` are wired via launch remappings (DES-001).
- C4 views: [docs/architecture/c4/level3_component_autonomy.md](../architecture/c4/level3_component_autonomy.md), [level3_component_navigation.md](../architecture/c4/level3_component_navigation.md).

## Design decisions (fixed)

| # | Decision | Choice | Rejected alternative |
|---|---|---|---|
| D1 | Survey spec carrier | **Extend `Mission.msg`** with survey fields (below) | New `SurveySpec.msg` — extra type + plumbing for one consumer; revisit only if a second mission type needs structured params |
| D2 | Mission lifecycle signal | **New `/mission_status` topic**, `drone_autonomy_msgs/MissionStatus` (below) | Overloading `/mission` republication — ambiguous arm/disarm semantics for the recorder (WP-2) |
| D3 | Sweep pattern | **Boustrophedon along the polygon's longest edge direction**, convex polygons only in v1 | Spiral/adaptive patterns — no benefit at survey altitudes, more failure modes |
| D4 | Camera model for footprint | **OAK-D RGB: HFOV 69°, VFOV 55°** as node parameters (`camera_hfov_deg`, `camera_vfov_deg`) | Reading camera_info at plan time — camera not necessarily running when planning |
| D5 | Non-convex/self-intersecting polygons | **Reject with error status** (v1) | Decomposition into convex cells — deferred until a mission needs it |

## Proposed design

### Message changes (task T1.2)

`msgs/ros2/Mission.msg` gains:

```
# Survey mission fields (mission_type == "survey"), CAP-001 / DES-003
geometry_msgs/Polygon survey_polygon   # vertices in local ENU frame, z ignored
float32 survey_altitude_m              # AGL altitude for the survey
float32 forward_overlap                # 0.0–1.0, default 0.75
float32 side_overlap                   # 0.0–1.0, default 0.60
```

New `msgs/ros2/MissionStatus.msg`:

```
std_msgs/Header header
string mission_id
string state        # "active" | "complete" | "aborted" | "rejected"
string detail       # human-readable reason (rejection cause, abort source)
```

### autonomy_node behavior (task T1.3) — `Implements: MAP-6`

1. New subscription `~/survey_request` (`drone_autonomy_msgs/Mission`) — the
   GCS-facing entry point (communication package forwards operator requests
   here; wiring beyond a launch remap is out of WP-1 scope).
2. Validation (reject → publish `MissionStatus{state: "rejected", detail}`):
   polygon has ≥ 3 vertices; no self-intersection (segment sweep test);
   `10.0 ≤ survey_altitude_m ≤ 120.0`; overlaps in `[0.30, 0.95]`.
3. On pass: publish the `Mission` on `~/mission` with `mission_type =
   "survey"`, unique `mission_id` (`survey_<utc-iso>`), then
   `MissionStatus{state: "active"}` on `~/mission_status`.
4. Publish `MissionStatus{state: "complete"}` when the trajectory-complete
   condition is met (v1: timer-armed placeholder hook `on_mission_complete()`
   — full completion tracking arrives with mission progress monitoring, out
   of CAP-001 scope), and `"aborted"` on any failsafe/mode-change input hook.

### navigation_node behavior (task T1.4) — `Implements: MAP-1`

New class `SurveyPlanner` (`src/navigation/src/survey_planner.cpp` + header in
`src/navigation/include/navigation/survey_planner.hpp` — pure geometry, no ROS
deps, unit-testable):

1. Footprint at altitude *h*: `W = 2·h·tan(HFOV/2)`, `H = 2·h·tan(VFOV/2)`.
2. Lane spacing `s_lane = W · (1 − side_overlap)`; capture spacing
   `s_cap = H · (1 − forward_overlap)`.
3. Sweep direction = orientation of the longest polygon edge. Lanes =
   intersections of the polygon with lines perpendicular-offset by `s_lane`,
   inset by `W/2` from the boundary; serpentine lane ordering.
4. Waypoints: lane endpoints plus intermediate points every `s_cap`; yaw
   aligned with lane direction; z = `survey_altitude_m`.
5. Output `drone_autonomy_msgs/Trajectory` (existing `nav_msgs/Path` payload)
   published on `~/trajectory` on receipt of a `mission_type == "survey"`
   mission. Non-survey missions keep current behavior.
6. Parameters (declared, with defaults): `camera_hfov_deg=69.0`,
   `camera_vfov_deg=55.0`, `max_lane_length_m=500.0` (reject larger),
   `boundary_margin_m=0.0`.

## Interfaces

| Direction | Topic | Type | QoS | Notes |
|---|---|---|---|---|
| in (autonomy) | `~/survey_request` | `drone_autonomy_msgs/Mission` | reliable, depth 10 | operator survey spec |
| out (autonomy) | `~/mission` | `drone_autonomy_msgs/Mission` | reliable, depth 10 | existing, remapped to `/mission` |
| out (autonomy) | `~/mission_status` | `drone_autonomy_msgs/MissionStatus` | reliable, **transient_local**, depth 10 | late-joining recorder must see current state |
| in (navigation) | `~/mission` | `drone_autonomy_msgs/Mission` | reliable, depth 10 | existing |
| out (navigation) | `~/trajectory` | `drone_autonomy_msgs/Trajectory` | reliable, depth 10 | existing, remapped per DES-001 |

Launch: add `/mission_status` remapping alongside the DES-001 remaps in
`launch/platform_core.launch.py` and `launch/platform.launch.py` (task T1.2).

## Safety impact

`src/navigation` is safety-critical (DO-178C review scope). Failure modes and
mitigations: degenerate polygon → rejected before dispatch (autonomy) and
defensively re-checked in `SurveyPlanner` (returns error, no trajectory);
altitude out of bounds → rejected; trajectory never exceeds the polygon hull
inset by the footprint margin (TS-03 asserts containment). Geofence/failsafe
interaction unchanged — the survey trajectory is consumed by the existing
control chain under the existing safety_node monitoring.

## Test strategy

TP-002: TS-01 (dispatch), TS-02 (rejection), TS-03 (geometry properties,
`Verifies: MAP-1`), TS-04 (SITL chain). Geometry tests run against
`SurveyPlanner` directly (no ROS graph needed).

## Alternatives considered

See Design decisions table. Additionally rejected: generating the lane plan in
`autonomy_node` (navigation owns spatial planning; autonomy owns mission
lifecycle — matches package boundaries and keeps the safety-critical surface
in one package).

## Open questions

None — decisions D1–D5 are fixed for WP-1. Changes go back to the designer
(capability skill), not to the implementing agent.
