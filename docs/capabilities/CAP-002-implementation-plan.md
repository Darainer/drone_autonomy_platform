# CAP-002 Implementation Plan — Pre-Mow Field Clearance Surveillance

**Status:** Proposed v2 — **execution-ready**: all DES docs + TP-003 written and per-task scoped
**Capability doc:** [CAP-002-premow-clearance.md](CAP-002-premow-clearance.md)
**Target spec:** [docs/architecture/target/CAP-002-premow-clearance.yaml](../architecture/target/CAP-002-premow-clearance.yaml)
**Gap report:** [docs/reports/gap_CAP-002.md](../reports/gap_CAP-002.md) (baseline 9/38)
**Design docs (written):** [DES-007](../design/DES-007-clearance-sensing-and-model.md) · [DES-008](../design/DES-008-clearance-mission-and-confirmation.md) · [DES-009](../design/DES-009-tracker-node.md) · [DES-010](../design/DES-010-clearance-recorder.md) · [DES-011](../design/DES-011-clearance-report-and-delivery.md)
**Test plan:** [TP-003](../test_plans/TP-003-premow-clearance.md) (full test specs TS-01..TS-19)
**Requirements:** STK-2, CLR-1..CLR-12

## Execution & review model

Identical to CAP-001's ([CAP-001-implementation-plan.md](CAP-001-implementation-plan.md)
§"Execution & review model") — human reviews this plan once and then each
WP's PR; the designer-class model resolves all design decisions in the DES
docs before execution; Sonnet-class executors / the agent workforce implement
resolved designs; Opus reviews every task; unmarked code doesn't count; every
task regenerates C4/gap/traceability artifacts. Harness constraints
(PR-level human gates, no `sim-test` plan steps) also carry over unchanged.

## Sequencing (all DES docs written — WPs are dispatchable)

1. WP-A starts immediately (long pole, no ROS dependency) in parallel with
   WP-B/C/D; WP-H's dataset/baseline prep also starts immediately.
   Interfaces between parallel WPs are fixed in the DES docs, not in each
   other's code: `Finding.msg` and `/mission_status` semantics (DES-008),
   `/tracked_objects` shape (DES-009), the findings-store
   `format_version: 1` (DES-010).
2. **Owner decision needed early: thermal sensor purchase** (DES-007 D3) —
   blocks WP-G and WP-H's hardware-dependent tasks (T1 dawn campaigns).
3. WP-E can start once WP-D's store fixture (F4) is checked in — it codes
   against DES-010's format, not WP-D's node.
4. WP-F after WP-A..E, WP-G/H merge **and** DES-006 (CAP-001 stream) has
   landed for the field portions; SITL needs only the merged WPs.

```
WP dispatch (this plan + DES-007..011 + TP-003 approved)
      ├─► WP-A (RGB clearance model; DES-007) ─────────────────┐
      ├─► WP-B (profiles + coverage + confirm; DES-008) ───────┤
      ├─► WP-C (tracker_node; DES-009) ────────────────────────┼─► WP-F (SITL e2e + day + dawn field trials; TP-003 TS-16..19)
      ├─► WP-D (clearance recorder; DES-010) ──────────────────┤        ▲
      ├─► WP-E (report + delivery; DES-011) ───────────────────┤        └ external prereqs: DES-006 bridge merged;
      ├─► WP-G (thermal integration; DES-007 §4) ──────────────┤           thermal sensor purchased (owner)
      └─► WP-H (thermal detection; DES-007 §4–5) ──────────────┘
```

---

## WP-A — RGB clearance detection model (DES-007 §1–2, §5) — **the long pole**

**Requirements:** CLR-2, CLR-9 · `safety_critical: false` · Agents: `ml-pipeline` / `ml-pipeline` · Branch `wpA-clearance-model`

Executes the dedicated plan in
[DES-007](../design/DES-007-clearance-sensing-and-model.md): operating-point
selection under the effective-GSD analysis (§1 — the 512×512 detector input
sets a fawn-size class floor and forces the OP-1 low-altitude vs OP-2 tiled
experiment), occlusion grades O0–O2 for the RGB day check (§2), and the
collection/labeling program (§5: staged decoy campaigns R1–R3 with staked
ground truth, occlusion/range-binned labeling and eval sets). Builds on the
deployed RF-DETR-Small stack and the Phase-2/3 process in
[perception_finetuning_plan.md](../architecture/perception_finetuning_plan.md).

### Clearance class set (starting point — final in/out list decided by A1 licensing + A3 baseline, written back to DES-007)

Subset + extension of the fine-tuning plan taxonomy: `person` (critical),
COCO livestock (`cow`, `sheep`, `horse`) and pets (`dog`, `cat`), new
`deer`/`goat` wildlife classes, vehicles (`car`, `truck`, `motorcycle`,
`bicycle`), machinery (`tractor`, `harvester`, `sprayer`). Class floor is
fawn-size ~0.4 m (DES-007 D7 — smaller targets are best-effort); generic
"unknown obstruction" is explicitly out (FE-4, change detection).

### Task breakdown

| Task | What | Deliverable | Markers |
|---|---|---|---|
| A1 | **Dataset survey + licensing matrix.** Evaluate the candidate sets below for class coverage at the clearance operating point; record license, commercial-use status, and download provenance. Academic-only sets are flagged *prototyping-only* per finetuning-plan §2.1.1 | `tools/clearance_model/DATASETS.md` + machine-readable provenance manifest | `Implements: CLR-9` (manifest tooling) |
| A2 | **Class taxonomy + label-space remap config.** Clearance label set (above) as config; per-dataset class remapping into it; COCO-JSON output | remap configs + conversion tool | — |
| A3 | **Ingest pipeline + evaluation harness + operating-point experiment.** Convert/merge candidate sets; build E-RGB/E-NEG (real imagery only, whole sites held out; **binned by range × occlusion grade × class** per DES-007 §5.4); metrics = per-class recall @ false-finding budget + AP50 + latency. **Evaluate the deployed COCO engine first** (baseline for CLR-2's Draft targets) and run the **OP-1 (15 m full-frame) vs OP-2 (tiled 25–30 m)** selection experiment (DES-007 §1.2, D1); write the selected operating point back into DES-007 | eval harness + baseline report + operating-point decision | — |
| A4 | **Collection campaigns + labeling execution per DES-007 §5:** R1 staged decoy flights (graded grass O0–O3, staked/RTK ground truth, 10–40 m altitude matrix), R2 partner-farm livestock, R3 hard negatives; CVAT dual-annotation with occlusion-grade attributes, auto-completeness checks on staged frames; synthetic fill (Isaac Sim/Unity, owned IP) for gap classes (`deer`, child-scale person) — never in eval sets | labeled dataset releases + provenance manifest entries | `Implements: CLR-9` |
| A5 | **Fine-tune RF-DETR-Small** (Phase-2 recipe: COCO weights → clearance dataset, W&B tracked, hyperparameter sweep per the finetuning plan); evaluate against the E-RGB gate (binned — aggregate-only numbers are not acceptance evidence, DES-007 D6); confusion-matrix review (deer↔dog, person↔shadow) | checkpoints + eval report | — |
| A6 | **Export + deployment.** Extend `scripts/export_rfdetr_onnx.py` to accept `pretrain_weights` (gap already noted in the finetuning plan); ONNX → TensorRT engine on Orin; `rfdetr_node` label-set/engine config for the clearance model; model card; latency check ≤25 ms | engine + deploy config + model card | `Implements: CLR-2` (label set/engine config in `src/perception`) |

### Candidate open-source datasets (license verification is task A1, not assumed)

| Dataset | Relevant content | Perspective | Expected license posture |
|---|---|---|---|
| VisDrone2019-DET | persons, vehicles | UAV | academic — **prototyping only** |
| Stanford Drone Dataset | pedestrians, cyclists | UAV nadir | non-commercial — prototyping only |
| HERIDAL / SARD | persons in wilderness (SAR) | UAV | research — verify |
| WAID (Wildlife Aerial Images from Drone) | sheep, cattle, horses, wildlife | UAV | published as open — verify exact terms |
| UAVDT / AU-AIR | vehicles | UAV | academic — verify |
| COWC | vehicles | aerial nadir | verify |
| Roboflow Universe (aerial livestock / person / deer sets) | livestock, persons, deer | UAV | per-set CC-BY/Apache — verify individually |
| Synthetic (Isaac Sim / Unity) | any class, esp. `deer`, children-scale persons | UAV | owned IP — no constraint |
| Own flight footage (per A4 protocol) | full ODD | UAV | owned IP — the production backbone |

Strategy mirrors finetuning-plan §2.1.1: prototype on the academic sets to
de-risk the recipe; the **deployed** model trains only on license-clean +
owned data (CLR-9 gate).

**WP PR gate (human):** gap lines CLR-2/CLR-9 (containers + behaviors) ✅;
eval report meets or re-baselines CLR-2 (a re-baseline is a requirement
change → back through the `requirements` skill); provenance manifest covers
every training image source.

```json
{
  "summary": "CAP-002 WP-A per DES-007: clearance dataset pipeline + RF-DETR fine-tune + TensorRT deployment. Validation: eval harness on held-out clearance set meets CLR-2 draft targets; deployed engine latency <= 25 ms",
  "safety_critical": false,
  "affected_packages": ["tools/clearance_model", "src/perception", "scripts"],
  "steps": [
    {"agent": "ml-pipeline", "task_queue": "ml-pipeline",
     "action": "A1-A3: dataset survey + licensing matrix + provenance manifest (Implements: CLR-9), label-space remap configs, ingest pipeline, held-out eval set, baseline eval of deployed COCO engine per DES-007", "depends_on": []},
    {"agent": "ml-pipeline", "task_queue": "ml-pipeline",
     "action": "A4: domain labeling plan + synthetic generation config per DES-007 (Implements: CLR-9)", "depends_on": [0]},
    {"agent": "ml-pipeline", "task_queue": "ml-pipeline",
     "action": "A5-A6: fine-tune RF-DETR-Small per DES-007 recipe, eval vs CLR-2, extend export_rfdetr_onnx.py for pretrain_weights, build engine, rfdetr_node clearance label-set config (Implements: CLR-2), model card", "depends_on": [1]}
  ]
}
```

*(GPU training runs (A5) execute on the training workstation per the
finetuning plan's infrastructure table; the `ml-pipeline` agent drives and
records them.)*

---

## WP-B — Clearance mission profiles + coverage + confirmation maneuver (DES-008)

**Requirements:** CLR-1, CLR-8, CLR-10 · **`safety_critical: true`** (navigation + flight behavior) · Branch `wpB-clearance-mission`

DES-008 fixes the message/profile design and the confirmation-maneuver spec
(triggering, altitude floor, budget, abort rules) from
[DES-007 §3.1](../design/DES-007-clearance-sensing-and-model.md).

| Task | Executor | What | Markers |
|---|---|---|---|
| B1 | `infra` / `orchestrator` | DES-008 "Message changes": `Mission.msg` clearance fields (areas + names, altitude, profile, confirm budget) **and new `Finding.msg`**; `/findings` launch remaps | — |
| B2 | `autonomy-dev` / `ros2-dev` | Clearance mission type in `autonomy_node`: validation (named polygons, altitude bounds, profile), per-profile dispatch, `/mission_status` lifecycle reuse (DES-003 pattern) | `Implements: CLR-1` + tests `Verifies: CLR-1` |
| B3 | `nav-dev` / `ros2-dev` | Per-profile parameterization of the MAP-1 coverage generator (DES-007 operating points: day RGB ≤4 cm/px effective, dawn thermal ≤4.5 cm/px native), ≥95% polygon coverage | `Implements: CLR-8` + tests `Verifies: CLR-8` |
| B4 | `autonomy-dev`, `nav-dev` / `ros2-dev` | **Confirmation maneuver** per DES-008: candidate-triggered waypoint insertion (descend to confirmation altitude over finding, re-image, resume), altitude-floor validation, per-mission budget, over-budget → unconfirmed finding | `Implements: CLR-10` + tests `Verifies: CLR-10` |
| B5 | `infra` / `orchestrator` | Regenerate C4/gap/traceability; commit artifacts | — |

**WP PR gate:** behaviors CLR-1, CLR-8, CLR-10 ✅; TS-01..04 + TS-11 green;
safety-critical diff attention on `src/navigation` and the maneuver's
altitude-floor/abort logic.

```json
{
  "summary": "CAP-002 WP-B per DES-008. SITL check for run_simulation: TS-04 — dispatch REF-AREAS day-profile clearance, expect one /trajectory passing TS-03 geometry within 5 s; TS-11 — inject 3 sub-threshold /findings, expect one clearance_confirm mission with 3 waypoints at 10 m after coverage-complete",
  "safety_critical": true,
  "affected_packages": ["msgs", "src/autonomy", "src/navigation"],
  "steps": [
    {"agent": "infra", "task_queue": "orchestrator",
     "action": "DES-008 message changes: clearance fields in Mission.msg, new Finding.msg, /findings remaps in both platform launch files; check in F1 REF-AREAS fixture", "depends_on": []},
    {"agent": "autonomy-dev", "task_queue": "ros2-dev",
     "action": "DES-008 autonomy_node behavior items 1-3 (clearance_request, clearance_validation.cpp, dispatch); mark Implements: CLR-1; author TS-01/TS-02 tests, mark Verifies: CLR-1", "depends_on": [0]},
    {"agent": "nav-dev", "task_queue": "ros2-dev",
     "action": "DES-008 navigation_node behavior: multi-area clearance planning with per-profile params + clearance_confirm pass-through with 8 m altitude floor; mark Implements: CLR-8; author TS-03 gtest suite, mark Verifies: CLR-8", "depends_on": [0]},
    {"agent": "autonomy-dev", "task_queue": "ros2-dev",
     "action": "DES-008 item 4 confirmation queue: /findings subscription, threshold/dedup/budget, nearest-neighbor tour dispatch as clearance_confirm, complete after tour; mark Implements: CLR-10; author TS-11 test, mark Verifies: CLR-10", "depends_on": [1, 2]},
    {"agent": "infra", "task_queue": "orchestrator",
     "action": "Regenerate C4 + gap + traceability, flip TP-003 CLR-1/8/10 rows to implemented, commit artifacts", "depends_on": [3]}
  ]
}
```

---

## WP-C — `tracker_node` (DES-009)

**Requirements:** CLR-3 · `safety_critical: false` · Branch `wpC-tracker`

| Task | Executor | What | Markers |
|---|---|---|---|
| C1 | `perception-dev` / `ros2-dev` | `tracker_node` in `src/perception` per DES-009: ByteTrack over `/detections` → `/tracked_objects` with persistent IDs (message/fields fixed in DES-009 — follows the perception_architecture.md planned design); COMP-4 50 ms budget | `Implements: CLR-3` + replay tests (one track per object across a recorded sequence) `Verifies: CLR-3` |
| C2 | `infra` / `orchestrator` | Launch integration (`full_stack.launch.py`), regenerate C4/gap/traceability | — |

**WP PR gate:** container `tracker_node` + flow `/detections`→tracker ✅;
TS-05 green; measured per-frame budget within COMP-4.

```json
{
  "summary": "CAP-002 WP-C per DES-009. SITL check for run_simulation: replay F5 through rfdetr stub -> tracker_node, assert 3 persistent ids, flicker suppressed, gap re-associated (TS-05)",
  "safety_critical": false,
  "affected_packages": ["src/perception"],
  "steps": [
    {"agent": "perception-dev", "task_queue": "ros2-dev",
     "action": "DES-009 C1: vendored byte_tracker.py + tracker_node.py per D1-D5 (min_hits gate, majority class, input-header passthrough); mark Implements: CLR-3; author TS-05 unit+replay tests, mark Verifies: CLR-3; check in F5 fixture", "depends_on": []},
    {"agent": "infra", "task_queue": "orchestrator",
     "action": "DES-009 C2: add tracker_node to rfdetr/full_stack launch files + /tracked_objects remaps; regenerate C4 + gap + traceability, flip TP-003 CLR-3 row", "depends_on": [0]}
  ]
}
```

---

## WP-D — `clearance_recorder_node` in new `src/surveillance` (DES-010)

**Requirements:** CLR-4, CLR-5 · `safety_critical: false` · Branch `wpD-clearance-recorder`

| Task | Executor | What | Markers |
|---|---|---|---|
| D1 | `infra` / `orchestrator` | Scaffold `src/surveillance` (CMakeLists, package.xml, launch include); check in fixtures F5 (replay bag) and F6 (geolocation scenes) per TP-003 (`Finding.msg` lands in WP-B task B1) | — |
| D2 | `perception-dev` / `ros2-dev` | Geolocation: `/tracked_objects` + `/oak/rgb/camera_info` + synced MAVROS pose (DES-004 sync approach) → WGS84 position; mow-polygon containment | `Implements: CLR-4` + tests `Verifies: CLR-4` |
| D3 | `perception-dev` / `ros2-dev` | Evidence capture: rolling pre-roll frame buffer, per-finding clip writer, structured findings store (format fixed in DES-010 — WP-E's input contract), `/findings` publication, `/mission`-driven arm/disarm | `Implements: CLR-5` + replay tests `Verifies: CLR-5` |
| D4 | `infra` / `orchestrator` | Regenerate C4/gap/traceability | — |

**WP PR gate:** container + recorder flows + behaviors CLR-4/CLR-5 ✅;
TS-06..08 green (one clip + record per injected in-area track, geolocated
within tolerance; schema + guards).

```json
{
  "summary": "CAP-002 WP-D per DES-010. SITL check for run_simulation: replay F5 with an armed clearance mission, assert findings.jsonl has one in-area finding per object with lat/lon within tolerance and clips with 1 s pre-roll (TS-07)",
  "safety_critical": false,
  "affected_packages": ["src/surveillance"],
  "steps": [
    {"agent": "infra", "task_queue": "orchestrator",
     "action": "Scaffold src/surveillance (CMakeLists, package.xml, launch include) per DES-010 D1; check in F6 geolocation fixtures", "depends_on": []},
    {"agent": "perception-dev", "task_queue": "ros2-dev",
     "action": "DES-010 geolocation: ray-cast per D2 + area containment, origin-fix pairing; mark Implements: CLR-4; author TS-06 gtest on F6, mark Verifies: CLR-4", "depends_on": [0]},
    {"agent": "perception-dev", "task_queue": "ros2-dev",
     "action": "DES-010 evidence pipeline: pre-roll ring buffer, clip writer, findings store format_version 1, track.csv, /findings publication, arm/disarm + storage guard per D3-D8; mark Implements: CLR-5; author TS-07/TS-08 tests, mark Verifies: CLR-5", "depends_on": [1]},
    {"agent": "infra", "task_queue": "orchestrator",
     "action": "Regenerate C4 + gap + traceability, flip TP-003 CLR-4/5 rows, commit artifacts", "depends_on": [2]}
  ]
}
```

---

## WP-E — Report pipeline + mobile delivery (DES-011)

**Requirements:** CLR-6, CLR-7 · `safety_critical: false` · Branch `wpE-clearance-report`

| Task | Executor | What | Markers |
|---|---|---|---|
| E1 | `ml-pipeline` / `ml-pipeline` | `tools/clearance_report`: findings store → per-area report (verdict, finding map overlay, clips, coverage-hole statement, RGB-limitation notice); runs unattended on the companion within the CLR-6 5-min budget | `Implements: CLR-6` + tests on a reference findings store `Verifies: CLR-6` |
| E2 | `comms-dev` / `ros2-dev` | `communication_node` `/findings` subscription → GCS live downlink; delivery per DES-011 decision (LTE upload + push preferred, local-WiFi/GCS fallback), registered-device config | `Implements: CLR-7` + tests `Verifies: CLR-7` |
| E3 | `infra` / `orchestrator` | Regenerate C4/gap/traceability; operator docs (registering areas + phone) | — |

**WP PR gate:** container `clearance_report_pipeline` + `/findings` flow +
behaviors CLR-6/CLR-7 ✅; TS-09/TS-10 green (reference store → report →
test endpoint + ntfy inside the 5-min budget).

```json
{
  "summary": "CAP-002 WP-E per DES-011. Validation: TS-09 build_report on fixture F4 (verdicts incl. never-CLEAR-with-holes) <= 4 min; TS-10 docker-compose delivery to local ntfy + ssh endpoint <= 5 min from complete",
  "safety_critical": false,
  "affected_packages": ["tools/clearance_report", "src/communication"],
  "steps": [
    {"agent": "ml-pipeline", "task_queue": "ml-pipeline",
     "action": "DES-011 E1: tools/clearance_report package (build_report, coverage, verdicts, render, report_on_complete) per D1-D3, D5; check in F4 reference store fixture; mark Implements: CLR-6; author TS-09 tests, mark Verifies: CLR-6", "depends_on": []},
    {"agent": "comms-dev", "task_queue": "ros2-dev",
     "action": "DES-011 E2: communication_node /findings subscription + optional UDP JSON forward; deliver.py upload + ntfy push per D4/D6; mark Implements: CLR-7; author TS-10 compose test, mark Verifies: CLR-7", "depends_on": [0]},
    {"agent": "infra", "task_queue": "orchestrator",
     "action": "Regenerate C4 + gap + traceability, flip TP-003 CLR-6/7 rows; operator docs for area/phone registration (delivery.yaml)", "depends_on": [1]}
  ]
}
```

---

## WP-G — Thermal sensor integration (DES-007 §4.1, §4.3)

**Requirements:** CLR-11 · `safety_critical: false` · Branch `wpG-thermal-integration`
**External prerequisite:** thermal sensor purchase (DES-007 D3 — **owner sign-off**).

| Task | Executor | What | Markers |
|---|---|---|---|
| G1 | designer + owner | Sensor selection per DES-007 D3 (Boson 640 default vs Hadron 640R if cross-modal labeling wins); procurement; mount/power/USB integration on the airframe (hardware doc under `docs/architecture/`) | — |
| G2 | `perception-dev` / `ros2-dev` | Driver bring-up publishing `/thermal/image_raw` + camera info; `EXTERNAL_SYSTEMS` registration; time sync against pose (DES-004 approach); launch profile | `Implements: CLR-11` + tests `Verifies: CLR-11` |
| G3 | `perception-dev` / `ros2-dev` | `clearance_recorder_node` thermal-clip support (colormapped rendering for the phone report) — lands after WP-D | — |
| G4 | `infra` / `orchestrator` | Regenerate C4/gap/traceability; commit artifacts | — |

**WP PR gate:** `thermal_cam` container + `/thermal/image_raw` flows ✅;
bench recording shows synced thermal frames + pose.

---

## WP-H — Thermal dawn-sweep detection (DES-007 §4.2, §5)

**Requirements:** CLR-12 · `safety_critical: false` · Branch `wpH-thermal-detection`
Dataset/baseline prep starts immediately; T1 campaign tasks wait on WP-G hardware.

| Task | Executor | What | Markers |
|---|---|---|---|
| H1 | `ml-pipeline` / `ml-pipeline` | Thermal data survey (BIRDSAI-class aerial TIR — licensing per A1 rules; ground-view sets for pretraining only) + E-THERM eval-set spec; heated-decoy inventory for T1 (body-temp heat pads, fawn-rescue practice) | — |
| H2 | `ml-pipeline` / `ml-pipeline` | **Classical baseline** (DES-007 D4): contrast/blob + altitude-derived size gating; evaluate on early T1 data against the E-THERM gate incl. O3 placements | — |
| H3 | `perception-dev` / `ros2-dev` | `thermal_detector_node` in `src/perception`: baseline detector (or model inference if H2 fails the gate) publishing modality-agnostic `/detections` | `Implements: CLR-12` + tests `Verifies: CLR-12` |
| H4 | `ml-pipeline` / `ml-pipeline` | **Only if H2 fails the E-THERM gate:** fine-tune on T1/T2 + license-clean thermal data; re-evaluate; export/deploy | — |
| H5 | `infra` / `orchestrator` | Regenerate C4/gap/traceability; commit artifacts | — |

**WP PR gate:** `thermal_detector_node` + `/detections` flow + behavior
CLR-12 ✅; E-THERM gate met **including fully concealed (O3) heated-decoy
placements**, binned by range × occlusion grade.

---

## WP-F — Validation (TP-003)

**Requirements:** STK-2, CLR-2, CLR-4..CLR-8, CLR-10, CLR-12 · Branch `wpF-clearance-validation`
**External prerequisites:** DES-006 bridge merged; WP-G hardware flying (dawn trial).

| Task | Executor | What |
|---|---|---|
| F1 | Sonnet session + `run_simulation` stage | SITL world with reference mow polygon + simulated targets (person/animal/vehicle models); scripted end-to-end for **both profiles**: dispatch → fly → findings (incl. an injected low-confidence candidate exercising the confirmation maneuver) → report; assertions per STK-2 (a)–(c), (e) |
| F2 | Sonnet session | **Day-check field trial** per TP-003: surrogate targets (incl. child-scale mannequin) at surveyed positions in graded grass, real flight, STK-2 (a)–(e) including report-on-phone timing |
| F3 | Sonnet session | **Dawn-sweep field trial** per TP-003: heated decoys incl. fully concealed O3 placements at dawn thermal-contrast conditions; STK-2 (f); results of F2/F3 filed as dated reports (`report` skill) |

**Capability complete when:** gap report reads 37/37, TP-003 green, both
field-trial reports filed.

---

## Design-decision status

All executor-facing decisions are **fixed** in DES-007..DES-011 (each doc's
"Design decisions (fixed)" table). Remaining open items are owner/designer
actions, not executor decisions:

- **Thermal sensor purchase** (DES-007 D3, §7) — owner; blocks WP-G and
  WP-H's T1 campaigns.
- Operating-point selection OP-1 vs OP-2 (DES-007 D1) — decided *by* WP-A
  task A3's experiment, written back to DES-007 by the designer.
- Decoy inventory, partner-farm access, dawn/low-AGL regulatory check
  (DES-007 §7) — operational prep for the campaigns and trials.

A task that cannot be completed without changing a fixed decision stops
and returns to the designer (`capability` skill) — it does not improvise.
