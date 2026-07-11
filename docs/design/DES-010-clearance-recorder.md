# DES-010 — Clearance Recorder: Geolocation, Evidence Capture & Findings Store

**Status:** Draft (approved at the CAP-002 WP-level human gate)
**Safety-critical:** no
**Capability:** CAP-002 · **Work package:** WP-D (recorder), consumed by WP-E (store format)
**Test specs:** TP-003 TS-06..TS-08

## Summary

New `clearance_recorder_node` in a new `src/surveillance` package turns
tracks into **findings**: it geolocates each `/tracked_objects` track
(pixel → WGS84 via camera intrinsics + synced pose), filters to the
mission's mow-area polygons, records a pre-rolled evidence clip and a
structured finding record per object, publishes live `Finding` messages on
`/findings`, and logs the flown track for coverage QA. The findings-store
directory is the contract input to the DES-011 report pipeline.

## Requirements addressed

| UID | How |
|---|---|
| CLR-4 | Ray-cast geolocation with synced pose, ≤5 m error budget (analysis below); polygon containment tagging |
| CLR-5 | Rolling pre-roll buffer → per-finding JPEG clip (≥1 s pre-roll, ≥3 s total) + finding record with class/confidence/track/position/UTC |

## Current state

No `src/surveillance` package. Inputs exist or arrive with sibling WPs:
`/tracked_objects` (DES-009), `/oak/rgb/image_raw` + `/oak/rgb/camera_info`
(depthai), `/mavros/local_position/pose` + `/mavros/global_position/global`
(MAVROS), `/mission` + `/mission_status` (DES-003/008). Patterns to reuse
from `src/mapping` (DES-004): ApproximateTime sync, transient_local status
subscription, storage guard, atomic manifest finalization.

## Design decisions (fixed)

| # | Decision | Choice | Rejected alternative |
|---|---|---|---|
| D1 | Language/package | **C++** `src/surveillance` (rclcpp) — same rationale as DES-004: sibling idioms, QoS patterns, smoke tooling | Python — image buffer management and sync idioms already exist in C++ next door |
| D2 | Geolocation model | **Flat-ground ray-cast**: bbox center → camera ray (intrinsics from `camera_info`, extrinsics = `camera_pitch_deg` mount param) → intersect local z=0 plane → ENU offset → WGS84 via the **origin fix** (GNSS fix paired with local pose at arm time, equirectangular approximation — fields are km-scale) | DEM-based terrain intersection — mow fields are locally flat; error budget holds without it (below) |
| D3 | Clip storage | **JPEG frame sequence + `clip.json`** per finding; the DES-011 report step renders phone-playable MP4 with ffmpeg on the companion | Onboard H.264 encoding in the node — encoder plumbing and failure modes in a ROS node for a job the report build does in one ffmpeg call |
| D4 | Clip framing | Rolling buffer of the raw stream downsampled to `clip_fps=10`; clip = `pre_roll_s=1.0` before the track's first confirmed frame → until track lost + 1 s, capped at `max_clip_s=10.0` | Full 30 Hz clips — 3× storage for no review value on a phone |
| D5 | Finding identity | One finding per tracker ID with geolocated position **inside a mow area**; `finding_id = <mission_id>/<track_id>`; confidence = running max; position = confidence-weighted mean over the track's geolocated points | Per-detection findings — duplicates; that's what CLR-3 exists to prevent |
| D6 | Outside-area tracks | Logged in `findings.jsonl` with `area_name: ""` and **not** published on `/findings` (param `publish_outside=false`) | Discarding — the report may still want "seen just outside area 2" context; publishing them — noise in the confirmation queue |
| D7 | Recording trigger & storage | Arm on `/mission` `mission_type=="clearance"` (also stays armed through the `clearance_confirm` follow-up mission — same `mission_id`); disarm on terminal `/mission_status`. Storage: `/data/clearance/<mission_id>/`, refuse to arm < 500 MB free, keep-last-5 offloaded rotation (DES-004 D7 semantics) | Always-on recording |
| D8 | Coverage QA source | Recorder logs `track.csv` (pose + active-sensor footprint at 1 Hz) — the report's coverage computation input | Deriving coverage from the planned trajectory — reports what was *intended*, not what was *flown*; misses aborts |
| D9 | Thermal clips (WP-G task G3) | Same pipeline, source topic `/thermal/image_raw`, stored as 16-bit PNG + colormapped JPEG preview per frame | Colormap-only — loses radiometric data for later analysis |

### Geolocation error budget (D2 — why ≤5 m holds)

At the day operating point (15 m AGL): attitude uncertainty ~2° → 0.5 m;
bbox-center vs. object centroid ≤ 0.5 m; sync-induced offset at 6 m/s with
50 ms slop ≤ 0.3 m (measured per pairing, as in DES-004); flat-ground
violation on mow fields ≤ 0.3 m; non-RTK GNSS origin 1–2.5 m (dominant).
RSS ≈ 2.7 m worst-case ≈ 3 m — inside the 5 m requirement with margin. At
the dawn point (22 m) the attitude term grows to ~0.8 m; still within
budget. The budget is verified synthetically (TS-06), not assumed.

## Proposed design

### Findings store (the WP-E input contract) — versioned, `format_version: 1`

```
clearance_<mission_id>/
  manifest.yaml        # format_version, mission_id, profile, areas (+names),
                       # altitude, start/end UTC, counts {findings, outside,
                       # dropped_sync}, origin fix, camera intrinsics,
                       # sha256 per file; finalized by atomic rename (DES-004)
  findings.jsonl       # one JSON object per finding (schema below)
  track.csv            # stamp_ns, x, y, z, qx..qw, lat, lon, footprint_w_m,
                       # footprint_h_m   (1 Hz, D8)
  clips/<track_id>/
    frame_<idx:04d>_<stamp_ns>.jpg   # or .png+.jpg for thermal (D9)
    clip.json          # finding_id, class, stamps, fps, pre_roll frames
```

`findings.jsonl` record: `finding_id, track_id, class_id, confidence,
latitude, longitude, area_name, first_stamp_ns, last_stamp_ns, n_frames,
clip_dir, status` (`"unconfirmed"`; upgraded to `"confirmed"` when the
track re-detects during the `clearance_confirm` mission at
`confirm_altitude`, matching by position within 5 m).

### clearance_recorder_node (tasks D2, D3) — `Implements: CLR-4`, `Implements: CLR-5`

- Sync: ApproximateTime(image, local pose), queue 30, slop 50 ms
  (DES-004 D4); GNSS attached most-recent-within-200 ms; camera_info
  latched at arm.
- Rolling buffer: synced (image, pose) pairs downsampled to `clip_fps`,
  ring of `pre_roll_s × clip_fps` entries.
- On `/tracked_objects` track with id *T* (header stamp matches a buffered
  pair): ray-cast (D2) → position; point-in-polygon over mission areas →
  `area_name`; create/update finding state (D5); write clip frames from
  the ring, then append live frames until track loss + 1 s (D4).
- Publish `Finding` on `/findings` (reliable, depth 50) on creation and on
  every status/confidence upgrade.
- Parameters: `camera_pitch_deg=-90.0`, `clip_fps=10.0`, `pre_roll_s=1.0`,
  `max_clip_s=10.0`, `publish_outside=false`, `output_dir=/data/clearance`,
  `jpeg_quality=90`, `min_free_mb=500`, `confirm_match_radius_m=5.0`.
- Failure behavior: write error → stop recording, ERROR diagnostic,
  finalize manifest `status: truncated` (DES-004 pattern); `/findings`
  publication continues (live path independent of storage path).

## Interfaces

| Direction | Topic | Type | QoS | Notes |
|---|---|---|---|---|
| in | `/tracked_objects` | `Detection2DArray` | reliable, depth 10 | DES-009 |
| in | `/oak/rgb/image_raw` | `Image` | best_effort, depth 5 | clip + sync source |
| in | `/oak/rgb/camera_info` | `CameraInfo` | reliable, depth 1 | intrinsics at arm |
| in | `/thermal/image_raw` | `Image` | best_effort, depth 5 | dawn profile (WP-G G3) |
| in | `/mavros/local_position/pose` | `PoseStamped` | best_effort, depth 30 | |
| in | `/mavros/global_position/global` | `NavSatFix` | best_effort, depth 10 | origin fix + track.csv |
| in | `/mission`, `/mission_status` | existing | reliable (+TL) | arm/disarm, areas |
| out | `/findings` | `drone_autonomy_msgs/Finding` | reliable, depth 50 | live findings (DES-008 D6, DES-011) |

## Safety impact

None (no control-path interaction); storage exhaustion handled locally,
recorder failure never affects the mission.

## Test strategy

TP-003: TS-06 (geolocation accuracy + polygon tagging, synthetic,
`Verifies: CLR-4`), TS-07 (clip pre-roll/duration/metadata + arm window,
replay, `Verifies: CLR-5`), TS-08 (store schema + manifest finalization +
storage guard, unit).

## Open questions

None — D1–D9 fixed for WP-D. Store format changes version through
`format_version` and return to the designer.
