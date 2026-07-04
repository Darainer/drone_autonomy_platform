# CAP-001 Implementation Plan — Visual Photogrammetry / Survey Mapping

**Status:** Proposed v2 (awaiting WP-level human approval)
**Capability doc:** [CAP-001-photogrammetry.md](CAP-001-photogrammetry.md)
**Target spec:** [docs/architecture/target/CAP-001-photogrammetry.yaml](../architecture/target/CAP-001-photogrammetry.yaml)
**Gap report:** [docs/reports/gap_CAP-001.md](../reports/gap_CAP-001.md)
**Test plan:** [docs/test_plans/TP-002-survey-mapping.md](../test_plans/TP-002-survey-mapping.md) (full test specs TS-01..TS-13)
**Design docs (written, per-task scoped):** [DES-003](../design/DES-003-survey-mission-coverage-trajectory.md) · [DES-004](../design/DES-004-survey-dataset-recording.md) · [DES-005](../design/DES-005-photogrammetry-pipeline.md)
**Requirements:** STK-1, MAP-1..MAP-8

v2 changes (designer review on commit 5dcf5cf): design decisions are now
**pre-scoped by the designer-class model** in DES-003/004/005 — every task
below points at the DES section and test specs (TS-*) it implements, so a
Sonnet-class executor implements a resolved design rather than making design
choices. The human reviews **at WP level only** (this plan once, then each
WP's PR); Opus reviews every task. The pipeline is **dual-target**: ground
station and onboard (Orin Nano class) post-flight execution (MAP-7, MAP-8).

---

## Execution & review model

| Role | Who | Responsibility |
|---|---|---|
| **WP-level review (human)** | Designer | Approves this plan (with its DES docs + test specs) once; thereafter reviews **only each WP's PR** against exit criteria. The human is not in the loop inside a WP. |
| **Design scoping** | Designer-class model (Opus/Fable) | All design decisions resolved in DES docs *before* execution (done — DES-003/004/005). Executors never make or re-open design decisions. |
| **Task execution** | Sonnet-class session or agent workforce (`scripts/submit_task.py`) | One task = one session/plan step, implementing the referenced DES section to the referenced TS pass criteria; must land the named markers |
| **Task review (Opus)** | Opus-class `/code-review` + `code-review` agent (`orchestrator` queue) | Every task diff, before it lands on the WP branch; findings return as `--rework` feedback |

**Rules binding every task:**

- The DES docs are the contract. A task that cannot be completed without a
  design change **stops** and goes back to the designer (`capability` skill);
  it does not improvise. DES/target/capability docs are not edited by
  executors (iteration-log appends excepted).
- Unmarked code does not count: land the `Implements:` markers named per task
  and the `Verifies:` + TS-id references named in TP-002.
- Every task finishes with the applicable regeneration commands
  (`generate_c4.py`, `check_architecture_gap.py`, `check_traceability.py`)
  and commits the regenerated artifacts.

**Harness constraints (unchanged from v1):**

- **Human gates are enforced at the PR.** `submit_task.py` submits
  `auto_approve=True`, so the workflow's safety-critical wait never fires;
  each WP develops on a feature branch and merges only via a human-approved
  PR — that PR **is** the WP gate.
- **No `sim-test` steps in `submit_task.py` plans.** The `simulation` queue
  serves only the built-in `run_simulation` stage, which runs automatically
  after the plan steps. Tests are authored in dev-agent steps; SITL scenarios
  + pass criteria (from the TS specs) go in the plan text.

**Definition of done — task:** diff on the WP branch, Opus review passed,
markers landed, referenced TS rows implemented and passing.
**Definition of done — WP:** gap-report lines ✅, TP-002 rows green, WP PR
approved and merged by the human.

---

## Dependency graph

```
        ┌─► WP-1 (mission type + coverage trajectory; DES-003) ─┐
plan+DES approval                                               ├─► WP-4 (end-to-end + field validation)
        ├─► WP-2 (survey recorder; DES-004)  ───────────────────┤
        └─► WP-3 (dual-target pipeline; DES-005) ───────────────┘
```

All three DES docs are approved together with this plan (single WP-level
human review), so WP-1/2/3 can start immediately and run in parallel:
WP-2 consumes only DES-003's *interface* (`/mission_status`), which is fixed
here, not WP-1's code; WP-3 consumes only DES-004's dataset format. WP-4
requires WP-1..3 merged.

---

## WP-1 — Survey mission type + coverage trajectory (DES-003)

**Requirements:** MAP-6, MAP-1 · **`safety_critical: true`** · Branch `wp1-survey-mission`

| Task | Executor (agent/queue) | Spec to implement | Markers / tests | Review |
|---|---|---|---|---|
| T1.1 | `infra` / `orchestrator` | DES-003 "Message changes": `Mission.msg` survey fields, new `MissionStatus.msg`, `/mission_status` launch remaps | — | Opus |
| T1.2 | `autonomy-dev` / `ros2-dev` | DES-003 "autonomy_node behavior" (validation rules, dispatch, lifecycle) | `Implements: MAP-6`; tests TS-01, TS-02 | Opus |
| T1.3 | `nav-dev` / `ros2-dev` | DES-003 "navigation_node behavior" (`SurveyPlanner`, params, publication) | `Implements: MAP-1`; tests TS-03 | Opus |
| T1.4 | `infra` / `orchestrator` | Regenerate C4/gap/traceability; README topic diagram; flip TP-002 rows; F1 fixtures checked in | — | Opus |

SITL scenario for the plan summary (built-in `run_simulation` stage): TS-04.

**WP PR gate (human):** gap behaviors MAP-6 + MAP-1 ✅; TS-01..04 implemented
and passing; diff review with safety-critical attention on `src/navigation`.

```json
{
  "summary": "CAP-001 WP-1 per DES-003. SITL check for run_simulation: TS-04 — dispatch REF-RECT survey, expect one /trajectory passing TS-03 geometry assertions within 5 s",
  "safety_critical": true,
  "affected_packages": ["msgs", "src/autonomy", "src/navigation"],
  "steps": [
    {"agent": "infra", "task_queue": "orchestrator",
     "action": "DES-003 message changes: survey fields in Mission.msg, new MissionStatus.msg, /mission_status remaps in both platform launch files", "depends_on": []},
    {"agent": "autonomy-dev", "task_queue": "ros2-dev",
     "action": "DES-003 autonomy_node behavior sections 1-4; mark Implements: MAP-6; author TS-01/TS-02 tests, mark Verifies: MAP-6", "depends_on": [0]},
    {"agent": "nav-dev", "task_queue": "ros2-dev",
     "action": "DES-003 navigation_node behavior: SurveyPlanner per steps 1-6; mark Implements: MAP-1; author TS-03 gtest suite, mark Verifies: MAP-1", "depends_on": [0]},
    {"agent": "infra", "task_queue": "orchestrator",
     "action": "Regenerate C4 + gap + traceability, update README topics diagram, commit F1 reference polygons fixture", "depends_on": [1, 2]}
  ]
}
```

---

## WP-2 — `survey_recorder_node` (DES-004)

**Requirements:** MAP-2, MAP-3 · `safety_critical: false` · Branch `wp2-survey-recorder`

| Task | Executor | Spec to implement | Markers / tests | Review |
|---|---|---|---|---|
| T2.1 | `infra` / `orchestrator` | Scaffold `src/mapping` (CMakeLists, package.xml, launch include); extend `scripts/smoke_test.sh` node list | — | Opus |
| T2.2 | `perception-dev` / `ros2-dev` | DES-004 "survey_recorder_node" + "Dataset format" (subscriptions, sync D4, trigger D6, storage D7, manifest) | `Implements: MAP-2`, `Implements: MAP-3`; tests TS-05, TS-06, TS-07 (writer side) | Opus |
| T2.3 | `infra` / `orchestrator` | F3 replay-bag fixture; regenerate C4/gap/traceability; README package list; flip TP-002 rows | — | Opus |

SITL scenario for the plan summary: TS-05/TS-06 replay assertions.

**WP PR gate (human):** `survey_recorder_node` container + all five target
flows ✅; TS-05..07 implemented and passing; dataset from replay validates.

---

## WP-3 — Dual-target photogrammetry pipeline (DES-005)

**Requirements:** MAP-4, MAP-5, **MAP-7, MAP-8** · `safety_critical: false` · Branch `wp3-photogrammetry-pipeline`

Scope per designer direction: the pipeline is **also executable on device**
(Orin Nano class) — `check` mode is the onboard post-flight consistency
check, `full` mode runs on both targets.

| Task | Executor | Spec to implement | Markers / tests | Review |
|---|---|---|---|---|
| T3.1 | `ml-pipeline` / `ml-pipeline` | DES-005 package skeleton + `dataset.py` + `verify_dataset.py` (DES-004 reader/validator) | tests TS-07 (reader side) | Opus |
| T3.2 | `ml-pipeline` / `ml-pipeline` | DES-005 `footprint.py` + `coverage.py` + `--mode check` (D3) | `Implements: MAP-5`, `Implements: MAP-7`; tests TS-09 | Opus |
| T3.3 | `ml-pipeline` / `ml-pipeline` | DES-005 `odm_runner.py` + `--mode full` (D1, D6) + multi-arch Dockerfile (D5) | `Implements: MAP-4`, `Implements: MAP-8`; tests TS-08 | Opus |
| T3.4 | `infra` / `orchestrator` | F2 sample-dataset fixture; `tools/photogrammetry/README.md` (both targets, runtimes); companion-image addition in `docker/`; regenerate gap/traceability; flip rows | — | Opus |

Jetson HIL runs TS-10 (check-mode ≤ 15 min) and TS-11 (onboard full mode) are
executed at the WP gate on hardware; their results attach to the WP PR.

**WP PR gate (human):** `photogrammetry_pipeline` container + behaviors
MAP-7/MAP-8 ✅; TS-08/09 green in CI; TS-10/11 HIL results attached.

---

## WP-4 — End-to-end validation (STK-1)

**Requirements:** MAP-5, MAP-7, STK-1 · Branch `wp4-e2e-validation` · Depends on WP-1..3 merged.

| Task | Executor | Spec to implement | Markers / tests | Review |
|---|---|---|---|---|
| T4.1 | Sonnet session | TS-12 end-to-end script (SITL survey → dataset → full mode → coverage gate) | `Verifies: MAP-5`; TS-12 | Opus |
| T4.2 | Sonnet session (`report` skill) | TS-13 field procedure doc + dated demonstration report (incl. onboard check ≤ 15 min) | — | Opus |
| T4.3 | `infra` / `orchestrator` | Final regeneration: gap report all-present; traceability green for MAP-1..8 | — | Opus |

**WP PR gate (human = capability complete):** gap report all elements
present; all TP-002 Test rows implemented + passing; TS-13 report filed with
STK-1 acceptance (a)–(d) met. Capability status flip is a designer action.

---

## Resolved design decisions (v1 open items → fixed)

| Decision | Resolution |
|---|---|
| Survey spec carrier | Extend `Mission.msg` — DES-003 D1 |
| Mission lifecycle / recording trigger | New `/mission_status` topic — DES-003 D2, DES-004 D6 |
| Dataset container format | Images + `poses.csv` + `manifest.yaml` — DES-004 D1 |
| Pose source | Raw MAVROS local pose + GNSS side-by-side — DES-004 D3 |
| SfM engine | OpenDroneMap (containerized, amd64+arm64) — DES-005 D1 |
| On-device execution (designer, commit-5dcf5cf review) | Dual-target pipeline: onboard `check` (MAP-7) + onboard `full` (MAP-8) — DES-005 D3/D5 |

## Change control

- Rework within a task: `--rework "<file/function-specific finding>"`.
- Any change to DES decisions, the target YAML, or requirement statements is
  a designer action via the `capability`/`requirements` skills; this plan and
  the TS specs are then updated to match before execution resumes.
