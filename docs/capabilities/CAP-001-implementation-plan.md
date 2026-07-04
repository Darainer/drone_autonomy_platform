# CAP-001 Implementation Plan — Visual Photogrammetry / Survey Mapping

**Status:** Proposed (awaiting human approval of WP-1 kickoff)
**Capability doc:** [CAP-001-photogrammetry.md](CAP-001-photogrammetry.md)
**Target spec:** [docs/architecture/target/CAP-001-photogrammetry.yaml](../architecture/target/CAP-001-photogrammetry.yaml)
**Gap report:** [docs/reports/gap_CAP-001.md](../reports/gap_CAP-001.md) — 6/14 present, 8 gaps
**Test plan:** [docs/test_plans/TP-002-survey-mapping.md](../test_plans/TP-002-survey-mapping.md)
**Requirements:** STK-1, MAP-1..MAP-6 (`docs/requirements/*.sdoc`)

This plan turns the four work packages in the CAP-001 handoff table into
concrete, agent-executable tasks. It is the working contract between the
designer (human), the executing agents, and the reviewers.

---

## Execution & review model

| Role | Who | Responsibility |
|---|---|---|
| **WP gate review** | Human (designer) | Approves each DES doc before implementation starts; reviews each completed WP against its exit criteria before the next WP begins |
| **Task execution** | Sonnet-class session (Claude Code on `claude-sonnet-5`) or the agent workforce via `scripts/submit_task.py` | One task = one session/plan; must land the `Implements:`/`Verifies:` markers named in the task |
| **Task review** | Opus-class session (`/code-review`) + the `code-review` agent on the `orchestrator` queue | Every task's diff is reviewed before merge; findings go back to the executor as `--rework` feedback |
| **Safety-critical review** | Opus review **and** human sign-off | Any task touching `src/navigation`, `src/control`, or `src/safety` (WP-1 T1.4) |

**Rules binding every task** (from the capability handoff contract):

- Executors implement to the approved DES doc — they do **not** edit
  `docs/architecture/target/*.yaml` or the capability doc (except appending to
  its iteration log). Target changes go back to the designer.
- Unmarked code does not count: the gap checker greps for `Implements:` and
  the traceability checker for `Verifies:`. Each task below names the markers
  it must land.
- Every task ends with the standing regeneration commands that apply to it:
  `python scripts/generate_c4.py` (node/topic changes),
  `python scripts/check_architecture_gap.py` (after merges),
  `python scripts/check_traceability.py` (requirement/test/marker changes) —
  and commits the regenerated artifacts.

**Definition of done for a task:** diff merged, review passed, named markers
present, regenerated reports committed, TP-002 rows it covers flipped from
`planned` to `implemented`.

**Definition of done for a WP:** its gap-report lines read ✅, its TP-002 rows
are green, and the human gate review is recorded in the capability doc's
iteration log.

---

## Dependency graph

```
DES-003 ──► WP-1 (mission type + coverage trajectory) ──► WP-4 (end-to-end validation)
DES-004 ──► WP-2 (survey recorder)                    ──►
DES-005 ──► WP-3 (photogrammetry pipeline)            ──►
```

- WP-1 and WP-2 implementation can proceed in parallel once their DES docs
  are approved; WP-2's recording trigger consumes the `/mission` semantics
  fixed in DES-003, so **DES-003 must be approved before DES-004 is finalized**.
- WP-3 depends only on the **dataset format decided in DES-004**, not on the
  onboard code — it can start as soon as DES-004 is approved and can be
  developed against a hand-built sample dataset.
- WP-4 requires all of WP-1..3 merged.

---

## WP-1 — Survey mission type + coverage trajectory generator

**Requirements:** MAP-6, MAP-1 · **Design doc:** DES-003 (to write, task T1.1)
**`safety_critical: true`** — touches `src/navigation`.
**Current state:** `autonomy_node` publishes a single hardcoded `waypoints`
mission; `navigation_node` receives missions but never publishes a
trajectory; `Mission.msg` has no survey fields (polygon/altitude/overlap).

| Task | Agent / queue | What | Markers | Review |
|---|---|---|---|---|
| T1.1 | Sonnet session (`design` skill) | Write **DES-003 — survey mission & coverage trajectory**: extend `Mission.msg` (survey polygon vertices, altitude, forward/side overlap) vs. new `SurveySpec.msg`; boustrophedon (lawnmower) lane generation from camera footprint (OAK-D FOV × altitude) and overlap; polygon validity limits; failure behavior for degenerate polygons | — | Opus review → **human approves DES-003 (gate)** |
| T1.2 | `infra` / `orchestrator` | Message change per DES-003 (msgs/ change comes first, per plan rules); update `msgs/CMakeLists.txt` | — | Opus |
| T1.3 | `autonomy-dev` / `ros2-dev` | Survey mission type in `autonomy_node`: accept a survey spec, validate it, dispatch `mission_type: "survey"` on `/mission`, publish mission start/end (recording arm/disarm semantics for WP-2) | `Implements: MAP-6` | Opus |
| T1.4 | `nav-dev` / `ros2-dev` | Coverage trajectory generator in `navigation_node`: on `mission_type == "survey"`, generate the lane pattern per DES-003 and publish `/trajectory` (feeds the existing DES-001 chain to control) | `Implements: MAP-1` | Opus **+ human (safety-critical)** |
| T1.5 | `sim-test` / `simulation` | Tests per TP-002 rows MAP-6, MAP-1: geometry property tests on the generated lanes (spacing vs. overlap, containment) + SITL survey dispatch test | `Verifies: MAP-6` / `Verifies: MAP-1` | Opus |
| T1.6 | `infra` / `orchestrator` | Regenerate C4 views, gap report, traceability matrix; update README node-topic diagram; flip TP-002 rows | — | Opus |

**Exit criteria (human gate):** gap-report behaviors "Survey mission type in
mission manager" and "Coverage-pattern trajectory generator in navigation" ✅;
TP-002 MAP-1/MAP-6 rows `implemented` and passing; SITL flies a polygon survey.

`submit_task.py` plan skeleton (finalize after DES-003 approval):

```json
{
  "summary": "Survey mission type + coverage trajectory generator (CAP-001 WP-1)",
  "safety_critical": true,
  "affected_packages": ["msgs", "src/autonomy", "src/navigation"],
  "steps": [
    {"agent": "infra", "task_queue": "orchestrator",
     "action": "Extend mission messages with survey spec fields per DES-003", "depends_on": []},
    {"agent": "autonomy-dev", "task_queue": "ros2-dev",
     "action": "Add survey mission type to autonomy_node per DES-003; mark Implements: MAP-6", "depends_on": [0]},
    {"agent": "nav-dev", "task_queue": "ros2-dev",
     "action": "Add boustrophedon coverage trajectory generator to navigation_node per DES-003; mark Implements: MAP-1", "depends_on": [0]},
    {"agent": "sim-test", "task_queue": "simulation",
     "action": "Implement TP-002 MAP-1/MAP-6 tests (lane spacing, containment, survey dispatch); mark Verifies: MAP-1, MAP-6", "depends_on": [1, 2]},
    {"agent": "infra", "task_queue": "orchestrator",
     "action": "Regenerate C4 + gap report + traceability matrix; update README topics diagram", "depends_on": [3]}
  ]
}
```

---

## WP-2 — `survey_recorder_node` (new `src/mapping` package)

**Requirements:** MAP-2, MAP-3 · **Design doc:** DES-004 (to write, task T2.1)
`safety_critical: false`.

| Task | Agent / queue | What | Markers | Review |
|---|---|---|---|---|
| T2.1 | Sonnet session (`design` skill) | Write **DES-004 — survey dataset recording & offload**: dataset container format (images + poses CSV + manifest vs. rosbag2 — decide here), pose/frame sync policy (`message_filters` approximate-time, explicit sync budget in ms), recording trigger (arm on survey `/mission` start, disarm on end, per DES-003 semantics), storage path/rotation, single-package offload procedure | — | Opus review → **human approves DES-004 (gate)** |
| T2.2 | `infra` / `orchestrator` | Scaffold `src/mapping` package (CMakeLists, package.xml, launch include in `platform.launch.py`); extend `scripts/smoke_test.sh` node list | — | Opus |
| T2.3 | `perception-dev` / `ros2-dev` | Implement `survey_recorder_node`: subscribe `/oak/rgb/image_raw`, `/mavros/local_position/pose`, `/mission`; write time-synced dataset per DES-004; implement the documented offload format | `Implements: MAP-2` and `Implements: MAP-3` | Opus |
| T2.4 | `sim-test` / `simulation` | Tests per TP-002 rows MAP-2, MAP-3: replay frames+pose, assert dataset frame count and per-frame sync error ≤ DES-004 budget; validate dataset against the documented manifest schema | `Verifies: MAP-2` / `Verifies: MAP-3` | Opus |
| T2.5 | `infra` / `orchestrator` | Regenerate C4/gap/traceability; add `src/mapping` to README package list; flip TP-002 rows | — | Opus |

**Exit criteria (human gate):** gap report shows `survey_recorder_node`
container and all three flows (`oakd →`, `mavros →`, `autonomy_node →`) ✅;
a dataset is produced in SITL and passes the schema test.

The CAP-001 doc contains the reference `submit_task.py` plan for this WP.

---

## WP-3 — `tools/photogrammetry` offboard pipeline + coverage QA

**Requirements:** MAP-4, MAP-5 · **Design doc:** DES-005 (to write, task T3.1)
`safety_critical: false`. Runs on the ground station, not the Jetson.

| Task | Agent / queue | What | Markers | Review |
|---|---|---|---|---|
| T3.1 | Sonnet session (`design` skill) | Write **DES-005 — post-flight reconstruction pipeline**: SfM/MVS engine choice (COLMAP vs. OpenDroneMap — decide here, with the ≤2 h STK-1(c) runtime budget on reference hardware as a selection criterion), input = DES-004 dataset format, outputs (georeferenced point cloud + textured mesh + coverage report), coverage-QA method (reconstruction footprint vs. survey polygon), fully unattended invocation (STK-1(d)) | — | Opus review → **human approves DES-005 (gate)** |
| T3.2 | `ml-pipeline` / `ml-pipeline` | Implement `tools/photogrammetry/` pipeline CLI: dataset in → 3D products out, single command, no interactive steps | `Implements: MAP-4` | Opus |
| T3.3 | `ml-pipeline` / `ml-pipeline` | Implement coverage-QA module: computes % of survey polygon covered by the reconstruction, emits machine-readable report | `Implements: MAP-5` | Opus |
| T3.4 | `sim-test` / `simulation` | Tests per TP-002 rows MAP-4, MAP-5: pipeline smoke on a small sample dataset (exit 0, products exist); coverage-QA unit test against synthetic geometry with a known hole | `Verifies: MAP-4` / `Verifies: MAP-5` | Opus |
| T3.5 | `infra` / `orchestrator` | `tools/photogrammetry/README.md` (usage, reference hardware, runtime); regenerate gap/traceability; flip TP-002 rows | — | Opus |

**Exit criteria (human gate):** gap report shows `photogrammetry_pipeline` ✅;
sample dataset → point cloud + mesh + coverage % with no manual steps.

---

## WP-4 — End-to-end validation (STK-1)

**Requirements:** MAP-5, STK-1 · **Test plan:** TP-002 (written — this plan's
companion). Depends on WP-1..3 merged.

| Task | Agent / queue | What | Markers | Review |
|---|---|---|---|---|
| T4.1 | `sim-test` / `simulation` | SITL end-to-end: reference polygon survey → dataset → WP-3 pipeline → assert coverage ≥ 95% automatically (TP-002 row STK-1/SITL) | `Verifies: MAP-5` (mission level) | Opus |
| T4.2 | Sonnet session (`report` skill) | Field-flight procedure + dated demonstration report in `docs/reports/` checking STK-1 acceptance (a)–(d) | — | **Human executes/validates the flight** |
| T4.3 | `infra` / `orchestrator` | Final regeneration: gap report must read **14/14 present**; traceability matrix green for MAP-1..6; capability doc status flip is then a **designer** action | — | Opus + human |

**Exit criteria (human gate = capability complete):** gap report 14/14, all
TP-002 Test rows implemented and passing, STK-1 demonstration report filed.

---

## Open design decisions (fixed in DES docs, not here)

Delegated by the capability doc; each is decided in exactly one DES doc:

| Decision | Decided in |
|---|---|
| Survey spec in `Mission.msg` vs. new message | DES-003 |
| Dataset container format (rosbag2 vs. images+CSV+manifest) | DES-004 |
| Pose source (raw MAVROS vs. VSLAM-fused) | DES-004 |
| Recording trigger semantics | DES-003 (mission side) / DES-004 (recorder side) |
| SfM engine (COLMAP vs. OpenDroneMap) | DES-005 |

DES numbers 003–005 are reserved by this plan (see `docs/design/README.md`
numbering rule: sequential, never reused).

## Change control

- Rework within a task: `submit_task.py --rework "<specific finding>"` or a
  follow-up Sonnet session; keep feedback pointed at the exact file/function.
- Anything that changes the target architecture (topics, containers, flows in
  the YAML) stops the task and goes back to the designer via the `capability`
  skill — then this plan is updated to match.
