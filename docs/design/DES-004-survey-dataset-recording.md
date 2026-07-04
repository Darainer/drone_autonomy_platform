# DES-004 — Survey Dataset Recording & Offload

**Status:** Draft (approved together with the CAP-001 implementation plan at the WP-level human gate)
**Safety-critical:** no
**Capability:** CAP-001 · **Work package:** WP-2
**Test specs:** TP-002 TS-05..TS-07

## Summary

New `survey_recorder_node` in a new `src/mapping` package records camera
frames with time-synchronized local pose and GNSS fix to an onboard dataset
in a documented, tool-agnostic format, armed/disarmed by the survey mission
lifecycle (DES-003 `/mission_status`). The dataset directory is the offload
unit and the direct input to the DES-005 pipeline on either execution target.

## Requirements addressed

| UID | How this design satisfies it |
|---|---|
| MAP-2 | Approximate-time sync of frame + pose (+ GNSS) within a 50 ms budget, recorded per frame |
| MAP-3 | Documented dataset format (manifest + checksums); the dataset directory is a single self-contained offload package |

## Current state

No `src/mapping` package, no recorder. Sources exist: `/oak/rgb/image_raw`
(depthai driver), `/mavros/local_position/pose`, `/mavros/global_position/global`
(MAVROS). Mission lifecycle arrives with DES-003.
C4: [level2_container.md](../architecture/c4/level2_container.md), [topics.md](../architecture/c4/topics.md).

## Design decisions (fixed)

| # | Decision | Choice | Rejected alternative |
|---|---|---|---|
| D1 | Dataset container | **Directory of JPEG images + `poses.csv` + `manifest.yaml`** | rosbag2 — opaque to SfM tools, forces a conversion step on every consumer, complicates MAP-7 onboard check |
| D2 | Image encoding | **JPEG quality 95** | PNG — 5–8× larger for negligible reconstruction gain at survey GSD |
| D3 | Pose source | **Raw MAVROS local pose + GNSS fix recorded side-by-side** | VSLAM-fused pose — not flight-proven on this platform yet; pipeline uses GNSS for georef, local pose for QA; fused pose can be added as a column later |
| D4 | Sync mechanism | **`message_filters::ApproximateTime`** (image, local pose), slop 50 ms; GNSS attached as most-recent-within-200 ms | Exact-time sync — sources are unsynchronized clocks/rates, exact match never fires |
| D5 | Capture policy | **Fixed rate, `capture_rate_hz` param, default 2.0** | Distance-triggered capture — needs velocity estimation in the recorder; DES-003 capture spacing already guarantees overlap at nominal speed |
| D6 | Recording trigger | **Arm on `/mission` with `mission_type=="survey"`; disarm on `/mission_status` `complete`/`aborted` for that `mission_id`** | Recording whenever airborne — wastes storage, couples recorder to flight state |
| D7 | Storage & rotation | **`/data/surveys/<dataset_id>/`; refuse to arm below 2 GiB free; keep-last-5 rotation of *offloaded* datasets only** | Unbounded retention — Orin eMMC/SD fills mid-mission |

## Proposed design

### Dataset format (task T2.3) — `Implements: MAP-3`

```
survey_<mission_id>/
  manifest.yaml     # format_version: 1, mission_id, polygon, altitude, overlaps
                    # (copied from Mission), start/end UTC, frame count,
                    # camera intrinsics (from camera_info), sha256 per file
  images/
    <frame_idx:06d>_<stamp_ns>.jpg
  poses.csv         # frame_idx, stamp_ns, x, y, z, qx, qy, qz, qw,
                    # pose_stamp_ns, lat, lon, alt_amsl, gnss_stamp_ns, sync_err_ms
```

`manifest.yaml` is written on disarm (atomic rename from `manifest.yaml.part`);
a dataset without a final manifest is invalid by definition (TS-07 checks
this). The directory is the offload unit — `scp -r`/`rsync` it; checksums in
the manifest make the transfer verifiable with `tools/photogrammetry/verify_dataset.py`
(part of WP-3).

### survey_recorder_node (task T2.3) — `Implements: MAP-2`

- Package `src/mapping` (C++, rclcpp — matches sibling packages; scaffold task T2.2).
- Subscriptions: `/oak/rgb/image_raw` (`sensor_msgs/Image`, sensor-data QoS),
  `/oak/rgb/camera_info` (latched capture at arm), `/mavros/local_position/pose`,
  `/mavros/global_position/global`, `/mission`, `/mission_status`
  (reliable, transient_local to catch state on late start).
- Sync: ApproximateTime(image, pose), queue 30, slop 50 ms; per-pair
  `sync_err_ms = |stamp_img − stamp_pose|` recorded in `poses.csv`; pairs
  with `sync_err_ms > 50` are dropped and counted (`dropped_sync` in manifest).
- Rate limiting to `capture_rate_hz` after sync (drop, don't queue).
- Parameters: `capture_rate_hz=2.0`, `output_dir=/data/surveys`,
  `jpeg_quality=95`, `min_free_gib=2.0`.
- Failure behavior: disk-full or write error → stop recording, publish
  diagnostic log at ERROR, finalize manifest with `status: truncated`.

## Interfaces

| Direction | Topic | Type | QoS | Notes |
|---|---|---|---|---|
| in | `/oak/rgb/image_raw` | `sensor_msgs/Image` | best_effort, depth 5 | sensor-data profile |
| in | `/oak/rgb/camera_info` | `sensor_msgs/CameraInfo` | reliable, depth 1 | intrinsics snapshot at arm |
| in | `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | best_effort, depth 30 | |
| in | `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | best_effort, depth 10 | georeferencing |
| in | `/mission` | `drone_autonomy_msgs/Mission` | reliable, depth 10 | arm trigger |
| in | `/mission_status` | `drone_autonomy_msgs/MissionStatus` | reliable, transient_local, depth 10 | disarm trigger |

No publishers except standard rosout diagnostics in v1.

## Safety impact

None (no control-path interaction). Storage exhaustion is handled locally
(D7); recorder failure never affects the mission — it logs and stops.

## Test strategy

TP-002: TS-05 (sync budget + completeness, `Verifies: MAP-2`), TS-06
(recording window), TS-07 (format schema + offload verifiability,
`Verifies: MAP-3`). Replay-based — no real camera needed.

## Alternatives considered

See decisions table. Python node was considered for speed of development;
rejected: JPEG encode + disk I/O at 2 Hz is fine in Python, but the sibling
packages, QoS idioms, and smoke tooling are C++ — consistency wins.

## Open questions

None — D1–D7 fixed for WP-2.
