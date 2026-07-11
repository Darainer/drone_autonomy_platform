# DES-009 — Persistent Object Tracking (`tracker_node`)

**Status:** Draft (approved at the CAP-002 WP-level human gate)
**Safety-critical:** no
**Capability:** CAP-002 · **Work package:** WP-C
**Test specs:** TP-003 TS-05

## Summary

New `tracker_node` in `src/perception` associates per-frame
`/detections` into persistent-ID tracks on `/tracked_objects` using
ByteTrack, so one physical object yields one downstream finding (CLR-3).
The node is modality-agnostic: it tracks whatever publishes `/detections`
(RGB `rfdetr_node` by day, `thermal_detector_node` at dawn — DES-007 D5),
realizing the integration planned in
[perception_architecture.md §Object Tracking](../architecture/perception_architecture.md).

## Requirements addressed

| UID | How |
|---|---|
| CLR-3 | ByteTrack association with persistent IDs; min-hits gate so transient false positives never become findings; COMP-4 budget |

## Current state

No tracker exists; `rfdetr_node` (`src/perception/src/rfdetr_node.py`)
publishes `vision_msgs/Detection2DArray` on `/detections` (bbox center/size
in original-image pixels; `results[0].hypothesis.class_id` = stringified
class int, `.score` = confidence). `perception_node` and
`detection_visualizer` consume `/detections` directly and are unaffected.

## Design decisions (fixed)

| # | Decision | Choice | Rejected alternative |
|---|---|---|---|
| D1 | Language | **Python** (`src/perception/src/tracker_node.py`) — sibling of `rfdetr_node.py`, same install pattern | C++ — no latency need (ByteTrack is ~1–2 ms/frame), Python keeps the perception detection path in one language |
| D2 | Tracker implementation | **Vendored minimal ByteTrack** (`src/perception/src/byte_tracker.py`: Kalman filter + IoU association via `scipy.optimize.linear_sum_assignment`; MIT/Apache attribution header) | pip `yolox`/`ultralytics` dependency — pulls a training framework onto the Jetson for ~300 lines of algorithm |
| D3 | Output message | **Reuse `vision_msgs/Detection2DArray`** with `Detection2D.id = str(track_id)`; hypothesis (class, score) carried over from the matched detection | New `TrackedObject.msg` — the standard message already has the `id` field; no schema to maintain |
| D4 | Track-confirmation policy | **`min_hits=3` consecutive associations before a track is first published**; lost tracks kept `track_buffer=30` frames for re-association | Publishing tentative tracks — every 1-frame false positive would reach the recorder and become a finding |
| D5 | Class handling | Association is class-agnostic (IoU-based, ByteTrack standard); the published class is the **majority class over the track's last 10 hits**, score = running max | Per-class independent trackers — fragments identity when the detector flickers between classes (deer↔dog) |
| D6 | Tile-seam merge (OP-2) | **Not in v1** (OP-1 baseline, DES-007 D1). If A3 selects OP-2, a pre-association NMS across tile boundaries is added here — reserved hook, do not implement now | — |

## Proposed design

### tracker_node (task C1) — `Implements: CLR-3`

- Subscribes `/detections` (`Detection2DArray`, best_effort, depth 5 —
  matches the sensor-derived stream).
- Per frame: convert detections to `[x1, y1, x2, y2, score]`; ByteTrack
  update (`track_thresh=0.5` split into high/low bands per the algorithm,
  `match_thresh=0.8`); emit confirmed tracks (D4) as `Detection2DArray` on
  `/tracked_objects` with `id`, majority class (D5), max score, and the
  **input message's header** (stamp/frame preserved for downstream sync).
- Empty frames still run `update()` (ages/expires tracks) and publish an
  empty array (downstream liveness).
- Parameters: `track_thresh=0.5`, `match_thresh=0.8`, `track_buffer=30`,
  `min_hits=3`, `frame_rate=30.0`.
- Per-frame wall time logged at DEBUG; 10 s throughput stat at INFO
  (pattern of `rfdetr_node._log_stats`).

### Launch integration (task C2)

Add `tracker_node` to `src/perception/launch/rfdetr.launch.py` and
`full_stack.launch.py` after `rfdetr_node`; top-level launch files gain the
`/tracked_objects` remap.

## Interfaces

| Direction | Topic | Type | QoS | Notes |
|---|---|---|---|---|
| in | `/detections` | `vision_msgs/Detection2DArray` | best_effort, depth 5 | from rfdetr_node or thermal_detector_node |
| out | `/tracked_objects` | `vision_msgs/Detection2DArray` | reliable, depth 10 | `Detection2D.id` = persistent track id |

## Safety impact

None — perception data path only; no control interaction.

## Test strategy

TP-003 TS-05 (`Verifies: CLR-3`): unit tests against `byte_tracker.py`
directly (deterministic synthetic sequences) + replay test through the node.
Key cases: one object → one ID across N frames; two crossing objects keep
distinct IDs; ≤`track_buffer`-frame detection gap re-associates to the same
ID; a 2-frame flicker never publishes (min_hits); per-frame update < 50 ms
(COMP-4) at 30 simulated tracks.

## Open questions

None — D1–D6 fixed for WP-C.
