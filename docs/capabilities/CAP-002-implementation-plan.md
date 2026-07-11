# CAP-002 Implementation Plan — Pre-Mow Field Clearance Surveillance

**Status:** Proposed v1 (awaiting owner approval of the plan; no implementation started)
**Capability doc:** [CAP-002-premow-clearance.md](CAP-002-premow-clearance.md)
**Target spec:** [docs/architecture/target/CAP-002-premow-clearance.yaml](../architecture/target/CAP-002-premow-clearance.yaml)
**Gap report:** [docs/reports/gap_CAP-002.md](../reports/gap_CAP-002.md) (baseline 9/37)
**Design docs:** [DES-007](../design/DES-007-clearance-sensing-and-model.md) (sensing/model/data — **written**, owner review with this plan); DES-008..DES-011 reserved, authored by the designer after plan approval — no WP executes against an unwritten DES
**Test plan:** TP-003 reserved (authored with the remaining DES docs)
**Requirements:** STK-2, CLR-1..CLR-12

## Execution & review model

Identical to CAP-001's ([CAP-001-implementation-plan.md](CAP-001-implementation-plan.md)
§"Execution & review model") — human reviews this plan once and then each
WP's PR; the designer-class model resolves all design decisions in the DES
docs before execution; Sonnet-class executors / the agent workforce implement
resolved designs; Opus reviews every task; unmarked code doesn't count; every
task regenerates C4/gap/traceability artifacts. Harness constraints
(PR-level human gates, no `sim-test` plan steps) also carry over unchanged.

## Sequencing after plan approval

1. [DES-007](../design/DES-007-clearance-sensing-and-model.md) is written
   and reviewed with this plan. Designer authors DES-008..DES-011 + TP-003
   (one PR, second WP-level human review — per the handoff contract these
   fix every interface and design decision the executors need).
2. WP-A starts immediately (long pole, no ROS dependency) in parallel with
   WP-B/C/D; WP-H's dataset/baseline prep also starts immediately.
3. **Owner decision needed early: thermal sensor purchase** (DES-007 D3) —
   blocks WP-G and WP-H's hardware-dependent tasks (T1 dawn campaigns).
4. WP-E after DES-010's findings-store format is approved (not after WP-D
   code).
5. WP-F after WP-A..E, WP-G/H merge **and** DES-006 (CAP-001 stream) has
   landed for the field portions; SITL needs only the merged WPs.

```
plan approval (incl. DES-007) ─► DES-008..011 + TP-003 (designer)
      ├─► WP-A (RGB clearance model; DES-007) ─────────────────┐
      ├─► WP-B (profiles + coverage + confirm; DES-008) ───────┤
      ├─► WP-C (tracker_node; DES-009) ────────────────────────┼─► WP-F (SITL e2e + day + dawn field trials; TP-003)
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
| B1 | `infra` / `orchestrator` | `Mission.msg` clearance fields (area-set, altitude, **mission profile** `dawn_thermal_sweep`/`day_rgb_check`, confirmation budget) per DES-008; launch remaps | — |
| B2 | `autonomy-dev` / `ros2-dev` | Clearance mission type in `autonomy_node`: validation (named polygons, altitude bounds, profile), per-profile dispatch, `/mission_status` lifecycle reuse (DES-003 pattern) | `Implements: CLR-1` + tests `Verifies: CLR-1` |
| B3 | `nav-dev` / `ros2-dev` | Per-profile parameterization of the MAP-1 coverage generator (DES-007 operating points: day RGB ≤4 cm/px effective, dawn thermal ≤4.5 cm/px native), ≥95% polygon coverage | `Implements: CLR-8` + tests `Verifies: CLR-8` |
| B4 | `autonomy-dev`, `nav-dev` / `ros2-dev` | **Confirmation maneuver** per DES-008: candidate-triggered waypoint insertion (descend to confirmation altitude over finding, re-image, resume), altitude-floor validation, per-mission budget, over-budget → unconfirmed finding | `Implements: CLR-10` + tests `Verifies: CLR-10` |
| B5 | `infra` / `orchestrator` | Regenerate C4/gap/traceability; commit artifacts | — |

**WP PR gate:** behaviors CLR-1, CLR-8, CLR-10 ✅; SITL: dispatching each
profile yields a `/trajectory` passing DES-008 geometry/GSD assertions, and
an injected low-confidence candidate triggers exactly one bounded
confirmation maneuver. Safety-critical diff attention on `src/navigation`
and the maneuver's altitude/abort logic.

---

## WP-C — `tracker_node` (DES-009)

**Requirements:** CLR-3 · `safety_critical: false` · Branch `wpC-tracker`

| Task | Executor | What | Markers |
|---|---|---|---|
| C1 | `perception-dev` / `ros2-dev` | `tracker_node` in `src/perception` per DES-009: ByteTrack over `/detections` → `/tracked_objects` with persistent IDs (message/fields fixed in DES-009 — follows the perception_architecture.md planned design); COMP-4 50 ms budget | `Implements: CLR-3` + replay tests (one track per object across a recorded sequence) `Verifies: CLR-3` |
| C2 | `infra` / `orchestrator` | Launch integration (`full_stack.launch.py`), regenerate C4/gap/traceability | — |

**WP PR gate:** container `tracker_node` + flow `/detections`→tracker ✅;
replay test green; measured per-frame budget within COMP-4.

---

## WP-D — `clearance_recorder_node` in new `src/surveillance` (DES-010)

**Requirements:** CLR-4, CLR-5 · `safety_critical: false` · Branch `wpD-clearance-recorder`

| Task | Executor | What | Markers |
|---|---|---|---|
| D1 | `infra` / `orchestrator` | Scaffold `src/surveillance` (CMakeLists, package.xml, launch include); `/findings` message definition in `msgs/` per DES-010 | — |
| D2 | `perception-dev` / `ros2-dev` | Geolocation: `/tracked_objects` + `/oak/rgb/camera_info` + synced MAVROS pose (DES-004 sync approach) → WGS84 position; mow-polygon containment | `Implements: CLR-4` + tests `Verifies: CLR-4` |
| D3 | `perception-dev` / `ros2-dev` | Evidence capture: rolling pre-roll frame buffer, per-finding clip writer, structured findings store (format fixed in DES-010 — WP-E's input contract), `/findings` publication, `/mission`-driven arm/disarm | `Implements: CLR-5` + replay tests `Verifies: CLR-5` |
| D4 | `infra` / `orchestrator` | Regenerate C4/gap/traceability | — |

**WP PR gate:** container + all 7 recorder flows + behaviors CLR-4/CLR-5 ✅;
replay of a recorded detection sequence produces one clip + record per
injected track, geolocated within tolerance.

---

## WP-E — Report pipeline + mobile delivery (DES-011)

**Requirements:** CLR-6, CLR-7 · `safety_critical: false` · Branch `wpE-clearance-report`

| Task | Executor | What | Markers |
|---|---|---|---|
| E1 | `ml-pipeline` / `ml-pipeline` | `tools/clearance_report`: findings store → per-area report (verdict, finding map overlay, clips, coverage-hole statement, RGB-limitation notice); runs unattended on the companion within the CLR-6 5-min budget | `Implements: CLR-6` + tests on a reference findings store `Verifies: CLR-6` |
| E2 | `comms-dev` / `ros2-dev` | `communication_node` `/findings` subscription → GCS live downlink; delivery per DES-011 decision (LTE upload + push preferred, local-WiFi/GCS fallback), registered-device config | `Implements: CLR-7` + tests `Verifies: CLR-7` |
| E3 | `infra` / `orchestrator` | Regenerate C4/gap/traceability; operator docs (registering areas + phone) | — |

**WP PR gate:** container `clearance_report_pipeline` + `/findings` flow +
behaviors CLR-6/CLR-7 ✅; end-to-end: reference findings store → report
rendered → delivered to a test device/endpoint inside the latency budget.

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

## Open decisions deferred to the remaining DES docs (designer, post-approval)

[DES-007](../design/DES-007-clearance-sensing-and-model.md) is written; its
remaining open items (§7) are the thermal sensor purchase (owner), decoy
inventory, partner-farm access, and the dawn/low-AGL regulatory check. The
operating-point choice (D1) is decided by the A3 experiment and written back.

| DES | Decisions it must fix |
|---|---|
| DES-008 | clearance `Mission.msg` fields (profile enum, confirmation budget), area registry storage, confirmation-maneuver trigger/altitude-floor/abort rules (from DES-007 §3.1) |
| DES-009 | `/tracked_objects` message shape, ByteTrack params, track-confirmation policy (frames before a finding), tile-seam duplicate merging (if OP-2 selected) |
| DES-010 | findings-store format (versioned — WP-E contract), clip container/codec (RGB + colormapped thermal), pre-roll buffer sizing vs Orin memory, sync budget |
| DES-011 | delivery transport (LTE push vs fallback), report package format (self-contained HTML vs PDF+GeoJSON), notification mechanism, device registration |
