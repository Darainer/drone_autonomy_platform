# DES-011 — Clearance Report Generation & Mobile Delivery

**Status:** Draft (approved at the CAP-002 WP-level human gate)
**Safety-critical:** no
**Capability:** CAP-002 · **Work package:** WP-E
**Test specs:** TP-003 TS-09, TS-10

## Summary

New offboard/companion package `tools/clearance_report` builds, without
manual intervention and within 5 minutes of mission completion, a
phone-reviewable clearance report from a DES-010 findings store — per-area
verdicts, a findings map, rendered clips, explicit coverage holes, and the
modality limitation note — and delivers it to the farmer's registered
mobile device with a push notification. `communication_node` gains the live
`/findings` downlink.

## Requirements addressed

| UID | How |
|---|---|
| CLR-6 | `build_report.py`: findings store → report package; verdict rules with coverage-hole computation from `track.csv`; ≤5 min budget |
| CLR-7 | `deliver.py`: upload + ntfy push to registered device; `report_on_complete.py` automation; local-WiFi fallback |

## Current state

`tools/photogrammetry` (DES-005) establishes the offboard-package pattern
(pip-installable, CLI, unattended, companion-executable).
`communication_node` (`src/communication/src/communication_node.cpp`) is a
logging stub (its `/attitude_command` subscription is removed by DES-006).
Findings store format: DES-010 (`format_version: 1`).

## Design decisions (fixed)

| # | Decision | Choice | Rejected alternative |
|---|---|---|---|
| D1 | Report package format | **Self-contained static bundle**: `index.html` (offline-viewable, embedded map SVG + finding cards), `verdicts.json`, `findings.geojson`, `clips/*.mp4` (ffmpeg-rendered from DES-010 JPEG sequences), zipped as `clearance_<mission_id>_report.zip` | PDF — clips can't embed; server-rendered page — requires connectivity to review in the field |
| D2 | Verdict rules | Per area: **`CLEAR`** iff 0 in-area findings **and** coverage ≥ 95%; **`FINDINGS(n)`** if n>0 (unconfirmed count shown separately); **`COVERAGE_INCOMPLETE`** if coverage < 95% (never `CLEAR` with holes — an unseen area is not a clear area). Every report carries the modality note (RGB: "surface-visible objects, up to ~70% vegetation occlusion"; thermal: "warm-bodied animals; full dense canopy residual risk") | Aggregate mission verdict — the farmer mows per field |
| D3 | Coverage computation | Union of per-sample sensor footprints from `track.csv` (D8 of DES-010) intersected with each area polygon (shapely); holes emitted as GeoJSON polygons into the report map | Planned-trajectory coverage — reports intent, not what was flown |
| D4 | Delivery transport | **Primary: upload `report.zip` + `verdicts.json` to a configured endpoint** (`rsync`/`scp` over the LTE/Tailscale link, telemetry.md Phase 2) **and push via [ntfy](https://ntfy.sh)** (self-hostable open-source push; farmer's phone runs the ntfy app subscribed to a per-operator topic; notification = per-area verdicts + report link). **Fallback: landing-WiFi download** — companion serves the report dir over HTTP on the field network; the notification (or GCS) shows the URL | FCM/APNs (vendor account + app build), SMS/email gateways (carrier deps) — v1 stays self-hostable; swap is confined to `deliver.py` |
| D5 | Trigger | **`report_on_complete.py`**: small rclpy subscriber (runs on the companion alongside the stack) watching `/mission_status`; on terminal state for a clearance mission it invokes `build_report.py` + `deliver.py` on the store. Also runnable manually | systemd path units — harder to test, equal function |
| D6 | Registration model | `~/.clearance/delivery.yaml` on the companion: `{operator, ntfy_url, ntfy_topic, upload_endpoint, fallback_http_port}` — provisioned once per vehicle/operator (operator docs, task E3) | In-mission delivery fields — credentials/endpoints don't belong in mission messages |
| D7 | `/findings` downlink | `communication_node` subscribes `/findings`, logs at INFO, and (param `gcs_host` non-empty) forwards JSON datagrams to `gcs_host:gcs_findings_port` (UDP) — GCS live visibility during flight; the phone report remains the product | MAVLink custom messages — heavier integration for a v1 monitor feed |

## Proposed design

### tools/clearance_report (task E1) — `Implements: CLR-6`

```
tools/clearance_report/
  pyproject.toml            # deps: shapely, jinja2, pyyaml; ffmpeg = system dep
  clearance_report/
    build_report.py         # CLI: build_report <store_dir> [-o out_dir]
    coverage.py             # track.csv footprints → per-area coverage % + hole polygons
    verdicts.py             # D2 rules → verdicts.json
    render.py               # jinja2 → index.html; ffmpeg clip rendering (10 fps JPEG seq → mp4; thermal: colormapped previews)
    deliver.py              # D4: upload + ntfy push; exit ≠ 0 on undelivered
    report_on_complete.py   # D5 trigger (rclpy)
  tests/  + fixtures/reference_store/   # checked-in DES-010 store (F4)
```

- `build_report.py` validates `manifest.yaml` (`format_version: 1`,
  checksums) before building; a truncated store (DES-010 failure path)
  builds a report marked `DATA INCOMPLETE`.
- Unattended: no stdin reads, exit 0 + `report.zip` on success (TS-09
  asserts on captured output, TS-08 pattern).
- Budget: end-to-end build ≤ 4 min on Orin-class arm64 for a 20-finding
  store (leaves ≥1 min of the CLR-6/7 window for delivery); ffmpeg
  invocations bounded by clip cap (DES-010 D4).

### communication_node change (task E2, with delivery config docs) — `Implements: CLR-7`

Subscribe `/findings` (reliable, depth 50); log; optional UDP JSON forward
(D7). Parameters: `gcs_host=""` (off), `gcs_findings_port=14560`.
(`Implements: CLR-7` marker lives on the delivery path in
`deliver.py` + this downlink; the behavior flips when both land.)

## Interfaces

| Direction | Interface | Notes |
|---|---|---|
| in (comm) | `/findings` (`Finding`, reliable, depth 50) | live downlink (D7) |
| in (report) | findings store dir | DES-010 `format_version: 1` |
| out (report) | `report.zip` + ntfy POST + upload | D1/D4 |

## Safety impact

None — post-mission/ground path only.

## Test strategy

TP-003: TS-09 (`Verifies: CLR-6`): build on the reference store (fixture
F4) → verdict correctness incl. a synthetic coverage-hole case and the
never-CLEAR-with-holes rule; budget timing. TS-10 (`Verifies: CLR-7`):
end-to-end against a local ntfy container + ssh endpoint → package
delivered + notification received ≤ 5 min from a simulated `complete`.

## Open questions

None — D1–D7 fixed for WP-E. Transport swaps (D4) are confined to
`deliver.py` and return to the designer only if the ntfy/upload model
itself changes.
