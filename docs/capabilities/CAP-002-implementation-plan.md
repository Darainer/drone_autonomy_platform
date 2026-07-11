# CAP-002 Implementation Plan — Pre-Mow Field Clearance Surveillance

**Status:** Proposed v1 (awaiting owner approval of the plan; no implementation started)
**Capability doc:** [CAP-002-premow-clearance.md](CAP-002-premow-clearance.md)
**Target spec:** [docs/architecture/target/CAP-002-premow-clearance.yaml](../architecture/target/CAP-002-premow-clearance.yaml)
**Gap report:** [docs/reports/gap_CAP-002.md](../reports/gap_CAP-002.md) (baseline 9/30)
**Design docs:** DES-007..DES-011 **reserved, to be authored by the designer after plan approval** — no WP executes against an unwritten DES
**Test plan:** TP-003 reserved (authored with the DES docs)
**Requirements:** STK-2, CLR-1..CLR-9

## Execution & review model

Identical to CAP-001's ([CAP-001-implementation-plan.md](CAP-001-implementation-plan.md)
§"Execution & review model") — human reviews this plan once and then each
WP's PR; the designer-class model resolves all design decisions in the DES
docs before execution; Sonnet-class executors / the agent workforce implement
resolved designs; Opus reviews every task; unmarked code doesn't count; every
task regenerates C4/gap/traceability artifacts. Harness constraints
(PR-level human gates, no `sim-test` plan steps) also carry over unchanged.

## Sequencing after plan approval

1. Designer authors DES-007..DES-011 + TP-003 (one PR, second WP-level
   human review — per the handoff contract these fix every interface and
   design decision the executors need).
2. WP-A starts immediately (long pole, no ROS dependency) in parallel with
   WP-B/C/D.
3. WP-E after DES-010's findings-store format is approved (not after WP-D
   code).
4. WP-F after WP-A..E merge **and** DES-006 (CAP-001 stream) has landed for
   the field portion; the SITL portion needs only the merged WPs.

```
plan approval ─► DES-007..011 + TP-003 (designer)
      ├─► WP-A (clearance model; DES-007) ────────────────┐
      ├─► WP-B (mission type + coverage; DES-008) ─────────┤
      ├─► WP-C (tracker_node; DES-009) ────────────────────┼─► WP-F (SITL e2e + field validation; TP-003)
      ├─► WP-D (clearance recorder; DES-010) ──────────────┤        ▲
      └─► WP-E (report + delivery; DES-011) ───────────────┘        └ external prereq: DES-006 bridge merged
```

---

## WP-A — Clearance detection model (DES-007) — **the long pole**

**Requirements:** CLR-2, CLR-9 · `safety_critical: false` · Agents: `ml-pipeline` / `ml-pipeline` · Branch `wpA-clearance-model`

Builds on the deployed RF-DETR-Small stack
([perception_architecture.md](../architecture/perception_architecture.md))
and the Phase-2/3 process in
[perception_finetuning_plan.md](../architecture/perception_finetuning_plan.md),
specialized to the clearance ODD: grassland/mow-height vegetation, 20–40 m
AGL, nadir→45° oblique, and a class set where **misses are the costly
failure** (person, animals) rather than mAP-balanced detection.

### Clearance class set (starting point — finalized in DES-007)

Subset + extension of the fine-tuning plan taxonomy: `person` (critical),
COCO livestock (`cow`, `sheep`, `horse`) and pets (`dog`, `cat`), new
`deer`/`goat` wildlife classes, vehicles (`car`, `truck`, `motorcycle`,
`bicycle`), machinery (`tractor`, `harvester`, `sprayer`). Generic "unknown
obstruction" is explicitly out (FE-4, change detection).

### Task breakdown

| Task | What | Deliverable | Markers |
|---|---|---|---|
| A1 | **Dataset survey + licensing matrix.** Evaluate the candidate sets below for class coverage at the clearance operating point; record license, commercial-use status, and download provenance. Academic-only sets are flagged *prototyping-only* per finetuning-plan §2.1.1 | `tools/clearance_model/DATASETS.md` + machine-readable provenance manifest | `Implements: CLR-9` (manifest tooling) |
| A2 | **Class taxonomy + label-space remap config.** Clearance label set (above) as config; per-dataset class remapping into it; COCO-JSON output | remap configs + conversion tool | — |
| A3 | **Ingest pipeline + evaluation harness.** Convert/merge candidate sets; build the *held-out clearance eval set* (real aerial imagery only, stratified by site to prevent leakage); metrics = per-class recall @ false-finding budget + AP50 + latency. **Evaluate the deployed COCO engine first** — the baseline number CLR-2's Draft targets are confirmed against | eval harness + baseline report | — |
| A4 | **Domain labeling plan** — the piece that puts the customer's domain in the data: capture protocol for own-flight footage over representative grassland (altitudes 20–40 m, nadir + oblique, grass heights/seasons, dawn/dusk when wildlife is active, harsh shadows); CVAT dual-annotation + adjudication per the finetuning-plan quality table; hard negatives (molehills, rocks, hay bales, fence posts, troughs, shadows); per-class minimum counts; synthetic-generation config (Isaac Sim/Unity, owned IP) to fill class gaps (e.g. `deer`) | labeling plan doc + synthetic config in `tools/clearance_model` | `Implements: CLR-9` |
| A5 | **Fine-tune RF-DETR-Small** (Phase-2 recipe: COCO weights → clearance dataset, W&B tracked, hyperparameter sweep per the finetuning plan); evaluate against CLR-2 targets on the held-out set; confusion-matrix review (deer↔dog, person↔shadow) | checkpoints + eval report | — |
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

## WP-B — Clearance mission type + surveillance coverage (DES-008)

**Requirements:** CLR-1, CLR-8 · **`safety_critical: true`** (navigation) · Branch `wpB-clearance-mission`

| Task | Executor | What | Markers |
|---|---|---|---|
| B1 | `infra` / `orchestrator` | `Mission.msg` clearance fields (area-set, altitude) per DES-008; launch remaps | — |
| B2 | `autonomy-dev` / `ros2-dev` | Clearance mission type in `autonomy_node`: validation (named polygons, altitude bounds), dispatch, `/mission_status` lifecycle reuse (DES-003 pattern) | `Implements: CLR-1` + tests `Verifies: CLR-1` |
| B3 | `nav-dev` / `ros2-dev` | Surveillance parameterization of the MAP-1 coverage generator: detection GSD/altitude/overlap profile, ≥95% polygon coverage | `Implements: CLR-8` + tests `Verifies: CLR-8` |
| B4 | `infra` / `orchestrator` | Regenerate C4/gap/traceability; commit artifacts | — |

**WP PR gate:** behaviors CLR-1, CLR-8 ✅; SITL: dispatching a reference
clearance mission yields a `/trajectory` passing DES-008 geometry/GSD
assertions. Safety-critical diff attention on `src/navigation`.

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

## WP-F — Validation (TP-003)

**Requirements:** STK-2, CLR-2, CLR-4..CLR-8 · Branch `wpF-clearance-validation`
**External prerequisite:** DES-006 bridge merged (field portion).

| Task | Executor | What |
|---|---|---|
| F1 | Sonnet session + `run_simulation` stage | SITL world with reference mow polygon + simulated targets (person/animal/vehicle models); scripted end-to-end: dispatch → fly → findings → report; assertions per STK-2 (a)–(c), (e) |
| F2 | Sonnet session | Field-trial procedure per TP-003: surrogate targets at surveyed positions, real flight, STK-2 (a)–(e) including report-on-phone timing; results filed as dated report (`report` skill) |

**Capability complete when:** gap report reads 30/30, TP-003 green, field
trial report filed.

---

## Open decisions deferred to the DES docs (designer, post-approval)

| DES | Decisions it must fix |
|---|---|
| DES-007 | final class set (incl. deer/children-scale handling), recall/false-finding targets vs A3 baseline, dataset in/out list after license vetting, synthetic-vs-real mix, eval-set composition |
| DES-008 | clearance `Mission.msg` fields, altitude/GSD profile numbers, area registry storage |
| DES-009 | `/tracked_objects` message shape, ByteTrack params, track-confirmation policy (frames before a finding) |
| DES-010 | findings-store format (versioned — WP-E contract), clip container/codec, pre-roll buffer sizing vs Orin memory, sync budget |
| DES-011 | delivery transport (LTE push vs fallback), report package format (self-contained HTML vs PDF+GeoJSON), notification mechanism, device registration |
