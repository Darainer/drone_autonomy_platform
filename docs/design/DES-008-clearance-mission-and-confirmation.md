# DES-008 — Clearance Mission Profiles, Coverage & Confirmation Maneuver

**Status:** Draft (approved at the CAP-002 WP-level human gate)
**Safety-critical:** yes — touches `src/navigation` and flight behavior (`src/autonomy`)
**Capability:** CAP-002 · **Work package:** WP-B
**Test specs:** TP-003 TS-01..TS-04, TS-11

## Summary

Adds the `clearance` mission type: the operator supplies a set of named
mow-area polygons and a mission profile (`day_rgb_check` or
`dawn_thermal_sweep`); `autonomy_node` validates and dispatches;
`navigation_node` plans per-area coverage with profile-specific
detection-GSD parameters (reusing `SurveyPlanner`), chained with transit
legs; after the coverage pass completes, `autonomy_node` plans a bounded
**confirmation tour** over low-confidence findings (CLR-10). All decisions
below are fixed — implementing agents do not re-open them.

## Requirements addressed

| UID | How |
|---|---|
| CLR-1 | Clearance fields in `Mission.msg`; per-profile validation + dispatch in `autonomy_node`; lifecycle on `/mission_status` |
| CLR-8 | Per-profile parameterization of the boustrophedon generator (DES-007 operating points), multi-area chaining |
| CLR-10 | Post-pass confirmation tour: low-altitude re-imaging waypoints, per-mission budget, unconfirmed reporting |

## Current state

- `autonomy_node` handles `survey` via `~/survey_request` → validation
  (`src/autonomy/src/survey_validation.cpp`) → `~/mission` +
  `~/mission_status` (`src/autonomy/src/autonomy_node.cpp`); completion is
  the DES-003 placeholder hook.
- `navigation_node` owns `SurveyPlanner`
  (`src/navigation/src/survey_planner.cpp`, pure geometry, unit-tested).
- `Mission.msg` carries the DES-003 survey fields; `MissionStatus.msg`
  exists.
- Flight execution: DES-006 bridge (waypoint upload, `/route_progress`) —
  Draft; until it lands, completion uses the DES-003 placeholder hook and
  end-to-end runs are SITL-only.

## Design decisions (fixed)

| # | Decision | Choice | Rejected alternative |
|---|---|---|---|
| D1 | Spec carrier | **Extend `Mission.msg`** with clearance fields (below) — same D1 rationale as DES-003 | New `ClearanceSpec.msg` |
| D2 | Multi-area handling | **One mission covers 1..8 areas**, planned in request order, chained by straight transit legs at the coverage altitude | One mission per area — N× dispatch/report overhead for the farmer's most common case (several fields per mowing day) |
| D3 | Profile parameterization | **Profile selects camera model + defaults** as `navigation_node` parameters: day = OAK-D 69°/55°, 15 m, side overlap 0.30; dawn = thermal 50°/40°, 22 m, side overlap 0.30 (DES-007 operating points; OP-1 baseline) | Per-mission free-form camera params — operator shouldn't restate sensor physics per flight |
| D4 | Confirmation timing | **Post-pass tour**: candidates are queued during coverage; when the coverage trajectory completes, autonomy dispatches one confirmation trajectory visiting candidates (nearest-neighbor order) within budget, then returns | Mid-route insertion — re-uploads the PX4 route per candidate (thrash), and chasing moving animals mid-pass degrades coverage guarantees |
| D5 | Confirmation mechanics | **Fly to (candidate x, y) at `confirm_altitude_m` (default 10 m), dwell `confirm_dwell_s` (default 3 s)**; standard full-frame inference at 10 m gives 2.7 cm/px effective — 1.5× better than the CLR-2 floor with zero detector changes. DES-007 §3.1's native-crop inference is a reserved optimization if confirm recall proves insufficient | Native-crop inference path in `rfdetr_node` now — new inference mode for a gain the altitude already provides |
| D6 | Candidate source | **`autonomy_node` subscribes `/findings`** (DES-010 `Finding.msg`); candidates = findings with `confidence < confirm_threshold` (param, default 0.80) and `status == "unconfirmed"` | Recorder-driven maneuver requests — mission authority stays in autonomy, one direction of control |
| D7 | Moved/absent targets | A confirmation visit that yields no re-detection leaves the finding **`unconfirmed`** (reported as such per CLR-10); it is never silently dropped or auto-dismissed | Auto-dismissal — an animal that moved is still a reason to look before mowing |
| D8 | Completion tracking | `/route_progress` (DES-006 D6) drives coverage-complete → tour → `complete` when the bridge lands; **until then the DES-003 placeholder hook** stands in (SITL tests drive statuses directly) | Blocking WP-B on DES-006 |

## Proposed design

### Message changes (task B1)

`msgs/ros2/Mission.msg` gains:

```
# Clearance mission fields (mission_type == "clearance"), CAP-002 / DES-008
string clearance_profile                 # "day_rgb_check" | "dawn_thermal_sweep"
geometry_msgs/Polygon[] clearance_areas  # 1..8 mow-area polygons, local ENU, z ignored
string[] clearance_area_names            # parallel to clearance_areas, unique, non-empty
float32 clearance_altitude_m             # 0 = profile default (day 15.0, dawn 22.0)
uint8 confirm_budget                     # max confirmation visits; 0 = default 10
```

New `msgs/ros2/Finding.msg` (defined here, produced by DES-010):

```
std_msgs/Header header
string finding_id        # "<mission_id>/<track_id>"
string mission_id
string area_name         # containing mow area ("" if outside all areas)
string class_id          # detector class (string, as in Detection2D results)
float32 confidence       # max track confidence so far
float64 latitude         # WGS84
float64 longitude
string status            # "unconfirmed" | "confirmed" | "dismissed"
```

Launch: add `/findings` remaps alongside the existing ones in
`launch/platform_core.launch.py` and `launch/platform.launch.py`.

### autonomy_node behavior (task B2) — `Implements: CLR-1`

1. New subscription `~/clearance_request` (`Mission`, reliable, depth 10) —
   GCS-facing entry point, sibling of `~/survey_request`.
2. Validation (`src/autonomy/src/clearance_validation.cpp`, sibling of
   `survey_validation.cpp`; reject → `MissionStatus{rejected, detail}`):
   profile ∈ {`day_rgb_check`, `dawn_thermal_sweep`}; 1–8 areas, each ≥3
   vertices, non-self-intersecting (reuse the survey segment-sweep test);
   `clearance_area_names` parallel, unique, non-empty;
   `clearance_altitude_m` is 0 or in `[10.0, 50.0]`; `confirm_budget` ≤ 20.
3. On pass: fill profile-default altitude if 0, `mission_id =
   clearance_<utc-iso>` (reuse `makeMissionId` with a type prefix argument),
   publish on `~/mission`, then `MissionStatus{active}`.
4. Confirmation queue (task B4) — `Implements: CLR-10`:
   subscribe `/findings`; while the mission is active, queue findings with
   `status=="unconfirmed" && confidence < confirm_threshold` (dedup by
   `finding_id`). On coverage-complete (D8): if the queue is non-empty,
   order by nearest-neighbor from the last coverage waypoint, truncate to
   `confirm_budget`, and publish a second `Mission` on `~/mission` with
   `mission_type = "clearance_confirm"`, same `mission_id`, and `waypoints`
   = one pose per candidate at `(x, y, confirm_altitude_m)` (yaw toward
   next leg; the existing `waypoints[]` field carries the tour — no new
   planner involvement). Publish `complete` only after the tour trajectory
   completes (or immediately if the queue is empty / budget 0).
5. Parameters: `confirm_threshold=0.80`, `confirm_altitude_m=10.0`,
   `confirm_dwell_s=3.0` (dwell realized as `capture_rate`-style dwell
   waypoint duplication — two identical waypoints `dwell·v` apart is v1;
   PX4 loiter-time items arrive with DES-006 refinement).

### navigation_node behavior (task B3) — `Implements: CLR-8`

1. On `mission_type == "clearance"`: for each area polygon in order, run
   `SurveyPlanner` with profile parameters (D3); concatenate lane plans,
   inserting a straight transit leg from each area's last waypoint to the
   next area's first at `clearance_altitude_m`.
2. On `mission_type == "clearance_confirm"`: pass-through planner — emit
   the mission's `waypoints[]` directly as a `Trajectory` (validation:
   altitude ≥ `min_confirm_altitude_m=8.0` parameter — the altitude floor;
   reject below).
3. New parameters (declared with defaults): `thermal_hfov_deg=50.0`,
   `thermal_vfov_deg=40.0`, `day_altitude_m=15.0`, `dawn_altitude_m=22.0`,
   `clearance_side_overlap=0.30`, `clearance_forward_overlap=0.30`,
   `min_confirm_altitude_m=8.0`.
4. Non-clearance missions keep current behavior. Convexity rule D5 of
   DES-003 applies per area (reject non-convex with error status, v1).

## Interfaces

| Direction | Topic | Type | QoS | Notes |
|---|---|---|---|---|
| in (autonomy) | `~/clearance_request` | `Mission` | reliable, depth 10 | operator entry point |
| in (autonomy) | `/findings` | `Finding` | reliable, depth 50 | candidate queue (D6) |
| out (autonomy) | `~/mission`, `~/mission_status` | existing | existing | + `clearance_confirm` dispatch |
| in/out (navigation) | `~/mission` → `~/trajectory` | existing | existing | clearance + confirm planning |

## Safety impact

`src/navigation` and flight behavior are safety-critical. The confirmation
tour is the lowest-AGL autonomous behavior on the platform (10 m):
mitigations — hard floor `min_confirm_altitude_m` enforced in navigation
(defense in depth vs. the autonomy-side validation), tour flown only after
coverage completes (no mid-route interruption), budget cap bounds added
flight time (battery: SAF-1 RTL overrides the tour like any mission — the
bridge stands down per DES-006 D5 and remaining candidates report
unconfirmed). Geofence/failsafe interaction otherwise unchanged.

## Test strategy

TP-003: TS-01 (dispatch, both profiles), TS-02 (rejection matrix), TS-03
(multi-area geometry, per-profile GSD, `Verifies: CLR-8`), TS-04 (SITL
chain), TS-11 (confirmation tour, `Verifies: CLR-10`).

## Open questions

None — D1–D8 fixed for WP-B. Changes return to the designer.
