# Designer Review — CAP-001 Visual Photogrammetry / Survey Mapping (2026-07-05)

**Reviewer:** designer-class model (Fable), at the request of the WP-level human.
**Scope:** the whole capability as it stands — WP-1 (merged, PR #23), WP-2 (merged,
PR #25), WP-3 (open, PR #26 @ `ab1aecf`), against CAP-001, DES-003/004/005,
TP-002, and the target spec `docs/architecture/target/CAP-001-photogrammetry.yaml`.
**Generated artifacts refreshed before review:** traceability matrix, C4 drift
check (up to date), gap report (18/18 on the WP-3 branch).

## Summary

The three build work packages are structurally sound and the WP-3 pipeline code
holds its design invariants (check mode is reconstruction-free; full mode is
unattended; coverage gates at 95%; ENU QA frame per DES-004 D3). However the
capability is **not yet field-exercisable end-to-end**: the CI smoke gate is
currently broken by a launch-composition miss (F-1), the operator has no path to
task a survey (F-2), and the checked-in F2 fixture cannot feed a real ODM run at
the TS-08/TS-11 gates (F-6). Root and launch READMEs have drifted from the code
(F-3, F-4). None of these invalidate the merged work; all are bounded fixes.

## Findings — required fixes

Severity: **B** = blocks a gate/CI today · **M** = blocks exercising the
capability or a WP gate · **m** = doc/hygiene drift.

| ID | Sev | Area | Finding | Required fix | Land where |
|---|---|---|---|---|---|
| F-1 | B | launch / CI | `scripts/smoke_test.sh` launches `platform_core.launch.py` and asserts `survey_recorder_node` is up, but `platform_core.launch.py` never includes the `mapping` package (only `platform.launch.py` does). The smoke gate fails as shipped; this is also why "the new launch script" is invisible in the core config. `mapping` is plain rclcpp with no Jetson deps, so its exclusion from core contradicts core's stated purpose (CI / hardware-independent testing) and starves WP-4's TS-12 SITL run of the recorder. | Add the `mapping` include to `platform_core.launch.py`; keep it in `platform.launch.py`. Update `launch/README.md` composition table accordingly. | PR #26 (infra fix) or immediate follow-up PR |
| F-2 | M | capability wiring | `/survey_request` — the MAP-6 operator entry point — has **no publisher** anywhere (C4 `topics.md` lists it as dangling). TS-01 exercises it by test-publishing directly. The target spec has no flow for it, so the gap checker cannot see the hole: today no operator/GCS path can dispatch a survey. | Designer action (this review): add a `gcs/communication → autonomy_node /survey_request` flow to the target YAML so the gap is machine-tracked, and scope a task (WP-4 or a small WP-3.5) wiring GCS→`/survey_request` through the communication package, with a TS. | Target spec (designer) + new task |
| F-3 | m | docs (root README) | Stale vs code: capability table still reads "**Planned, awaiting WP-1 kickoff** — gap 15/18" (reality: WP-1/2 merged, WP-3 in PR, 18/18 on the branch); the Node Topics diagram omits `survey_recorder_node` entirely (6 subscriptions, the WP-2 node); "`src/mapping` and `tools/` appear in the tree as the work packages land" is overtaken by events. | Refresh the capability row (status + gap count), add the recorder box to the Node Topics diagram, and mention `tools/photogrammetry` under Software Architecture. | PR #26 (infra fix) |
| F-4 | m | docs (launch/README.md) | Claims `platform.launch.py` = "all **six** subsystems" and its comparison table has no `mapping` row; `platform_core` described as five subsystems. Written before WP-2 landed the seventh package. | Add the `mapping` row to the table, correct subsystem counts, document core's composition after F-1. | PR #26 (infra fix), together with F-1 |
| F-5 | M | DES-004 D7 | The **keep-last-5 rotation of offloaded datasets** decided in D7 is unimplemented, untested (no TS row), and currently unimplementable as specified — nothing defines how a dataset is marked "offloaded". The 500 MB arm-guard half of D7 *is* implemented. | Designer decision needed: either (a) amend DES-004 to define an offload marker (e.g. `offloaded.stamp` written by the offload procedure / `verify_dataset.py --mark-offloaded`) and add a recorder-side rotation task + TS, or (b) formally descope rotation from v1 in DES-004's iteration log and rely on the 500 MB guard. Recommendation: (b) for v1 — the guard already prevents mission-blocking storage exhaustion; revisit with the offload tooling. | DES-004 amendment (designer) ± new task |
| F-6 | M | WP-3 fixtures / gates | The checked-in F2 (`survey_ref_rect_sample`) uses 8×8 px placeholder JPEGs while its manifest declares 640×480 intrinsics. Fine for check-mode CI (geometry uses manifest intrinsics only), but the **gate-time TS-08 (real ODM on Docker) and TS-11 (Jetson HIL) runs cannot reconstruct from 8×8 placeholders** — TP-002 F2 intends "synthetic renders or bench captures". Also, no validator flags the image-dims/intrinsics mismatch, which would equally hide a recorder bug. | (a) Extend `generate_sample_dataset.py` with a renders mode producing ODM-usable imagery at the declared resolution (textured/patterned ground so SfM can match features), used to produce the gate-run F2 variant (not necessarily checked in — document how the gate run obtains it); (b) add a cheap consistency check (warning in `verify_dataset.py` / check mode) when actual image dimensions ≠ manifest intrinsics width/height. | (a) before the WP-3 gate runs; (b) PR #26 or follow-up |
| F-7 | M | WP-3 ODM interface | `write_geo_txt()` writes bare `WGS84` as the geo.txt projection line. ODM's documented projection strings are PROJ/EPSG forms (e.g. `EPSG:4326`, `+proj=utm …`); bare `WGS84` is not among them and risks a rejected/misparsed geo file at the first real ODM run. | One-line fix: emit `EPSG:4326` (with a comment citing the ODM geo-file docs); adjust the unit test. Validated for real at the TS-08 Docker gate as already planned. | PR #26 |

## Deviations accepted (recorded, no action)

- **DES-003 T1.1 remap placement** — the `~/mission` / `~/mission_status` /
  `~/survey_request` remaps landed in the package-level `autonomy.launch.py`
  rather than "both platform launch files" as T1.1 literally read. Both
  top-level files include it, so the wiring is correct in both configurations;
  package-level is arguably the better home (single source). Accepted as-built.
- **Orthophoto bounding-box coverage (WP-3 v1)** — full-mode coverage QA uses
  the orthophoto valid-data bounding box and can overestimate over interior
  holes; check mode (per-frame footprint union) is the operational hole
  detector. Documented in `tools/photogrammetry/README.md`; hole-accurate
  mask→polygon extraction is a HIL-gate refinement.
- **Projected-CRS orthophotos unsupported (WP-3 v1)** — full mode raises an
  actionable error rather than comparing mismatched frames; resolved against
  real ODM output at the gates.
- **Nadir / no-lever-arm camera convention (WP-3)** — forced by DES-004
  recording body pose only; consistent with TP-002's footprint model. Accepted;
  a future camera-extrinsic column composes cleanly (documented in
  `footprint.py`).

## Reviewed and found sound

- WP-1: survey validation/dispatch and `SurveyPlanner` land their markers; the
  `/mission` chain is wired and C4-clean; TS-01..04 implemented.
- WP-2: recorder QoS/sync/trigger semantics match DES-004 D4/D6 (including
  transient_local on `/mission_status` both ends); 500 MB guard and
  mission-set capture rate present; TS-05..07 implemented (replay + unit);
  `smoke_test.sh` node list was extended (see F-1 for the launch-side miss).
- WP-3 (PR #26 @ `ab1aecf`): both Codex P1s fixed properly (real ODM output
  collection with the positional-arg invocation; ENU-frame coverage QA via the
  poses-derived affine). Check mode imports no reconstruction path; full mode
  is stdin-closed with zero prompts; exit codes gate at 95%; FE-1
  forward-compatibility honored via name-based reads; 67 pipeline tests green.
- Traceability tooling extension to `tools/` (WP-3 finish) matches DES-005's
  stated tracking model and is scoped correctly (behavior markers only).

## Verification & traceability state (regenerated today)

- Gap `CAP-001`: **18/18 present** on the WP-3 branch (15/18 on `main` until
  PR #26 merges).
- Matrix: MAP-1/2/3/4/5/6 ✅ verified; **MAP-7, MAP-8 🟡 planned** — their
  verification is TS-10/TS-11 (Jetson HIL) executed at the WP-3 gate with
  results attached to PR #26; TS-10's 15-minute budget is **unmeasured until
  that run** (the design argument — pure numpy/shapely — is sound but not
  evidence).
- Safety-critical rows: MAP-1 ⚠ is implemented + verified (TS-03/TS-04); no
  safety-critical row is uncovered.
- STK-1 remains open pending WP-4 (TS-12 e2e, TS-13 field demonstration).

## Recommended sequencing

1. **On PR #26 before merge:** F-7 (geo.txt `EPSG:4326`), F-1 (core launch +
   smoke gate), F-3/F-4 (README refresh), F-6(b) (dims-consistency warning).
2. **At the WP-3 gate:** produce the F-6(a) render-based F2 variant, run TS-08
   (Docker, real ODM), TS-10, TS-11 (HIL); attach results to PR #26.
3. **Designer actions (this review):** F-2 target-spec flow + task scoping;
   F-5 DES-004 amendment (recommend descope-with-rationale for v1).
4. **WP-4** proceeds once 1–3 are done; TS-12 additionally needs the simulated
   camera feed on `/oak/rgb/image_raw` in SITL (already in its TS).
