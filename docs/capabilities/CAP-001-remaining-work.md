# CAP-001 — Remaining Work to Realize the Capability

**Status:** reference plan, written at the WP-3 merge gate (PR #26, 2026-07-05).
**Baseline:** WP-1/2/3 merged — code-complete for MAP-1..8; gap report 18/18.
What remains is **evidence, wiring, validation, and hardware prep**, not core build.
Sources: [implementation plan](CAP-001-implementation-plan.md) (WP-4),
[designer review 2026-07-05](../reports/2026-07-05-CAP-001-designer-review.md)
(open findings F-2/F-5/F-6a), [TP-002](../test_plans/TP-002-survey-mapping.md).

Done means: TP-002 rows MAP-7/MAP-8/STK-1 green, TS-13 report filed with
STK-1(a)–(d) met, capability status flipped by the designer.

---

## A — WP-3 gate evidence (hardware/Docker runs the CI could not do)

| # | Item | Exit criterion | Owner |
|---|---|---|---|
| A1 | **Render-based F2 variant** (review F-6a): extend `tools/photogrammetry/tests/fixtures/generate_sample_dataset.py` with a mode that renders ODM-matchable imagery (textured ground, declared 640×480 resolution). The checked-in 8×8 F2 is check-mode-only and now triggers the dims warning by design. | `--mode full` on the variant reaches real SfM feature matching | Sonnet task |
| A2 | **TS-08 real run** — `--mode full` with real ODM (`opendronemap/odm:3.5.4`) on an x86_64 Docker host using A1's dataset. First real validation of the `EPSG:4326` geo.txt and the orthophoto CRS path. **Expected follow-up:** ODM commonly emits UTM (projected) orthophotos; v1 deliberately raises an actionable error on projected CRS — if hit, scope the small UTM→ENU extension in `odm_runner.py` (the affine-fit seam already exists). | Exit 0, D6 products, numeric `coverage_pct` | Human-triggered + Sonnet fix task if CRS extension needed |
| A3 | **TS-10 (MAP-7)** — Jetson Orin HIL: `--mode check` on a 1000-frame dataset (A1 generator, `rect_width≈610` produces ~1008 frames). The 15-min budget is argued, never measured. | Wall time ≤ 15 min, report complete, exit code gates at 95% | Human (Orin) |
| A4 | **TS-11 (MAP-8)** — Jetson Orin HIL: `--mode full` unattended on arm64; wall time recorded (documented, not gated). | Same product set as TS-08; run report filed | Human (Orin) |
| A5 | File A2–A4 results as a dated report in `docs/reports/`, link from TP-002, flip MAP-7/MAP-8 rows to implemented/passed, rerun traceability. | Matrix: MAP-7/MAP-8 ✅ verified | Sonnet task (`report` skill) |

## B — Capability wiring gaps (from designer review)

| # | Item | Exit criterion | Owner |
|---|---|---|---|
| B1 | **Operator tasking path (F-2)**: `/survey_request` has no publisher — no way to task a survey outside tests. Designer adds the `gcs → communication → /survey_request` flow to the target YAML; executor task wires it through the communication package (GCS/MAVLink side per DES-002 idioms) + a TS. | Gap report tracks and shows the flow ✅; survey dispatchable from GCS | Designer (target spec) + Sonnet task |
| B2 | **DES-004 D7 rotation (F-5)**: keep-last-5 rotation of offloaded datasets is unimplemented and "offloaded" is undefined. Designer decision — recommendation stands: **descope from v1** via DES-004 iteration-log note (500 MB guard already covers mission risk); revisit with offload tooling. | DES-004 amended; finding closed | Designer |

## C — WP-4 end-to-end validation (per implementation plan)

| # | Item | Exit criterion | Owner |
|---|---|---|---|
| C1 | **T4.1 / TS-12**: single e2e script — SITL survey (`platform_core` now includes the recorder) → dataset → `--mode full` → coverage gate. Needs a **simulated camera** publishing `/oak/rgb/image_raw` in SITL (only missing SITL piece). | `coverage_pct ≥ 95` asserted automatically, zero manual steps | Sonnet task |
| C2 | **T4.2 / TS-13**: field procedure doc + dated demonstration report (includes onboard check ≤ 15 min on site). | STK-1(a)–(d) evidenced | Sonnet (`report` skill) + human flight |
| C3 | **T4.3**: final regeneration — gap all-present on `main`, traceability green MAP-1..8, TP-002 all rows closed. | All artifacts green and committed | Sonnet task |

## D — Physical / hardware prerequisites (before A3–A4 and the field demo)

| # | Item | Why |
|---|---|---|
| D1 | **Camera intrinsics on real hardware**: the manifest snapshots `/oak/rgb/camera_info` at arm — verify the OAK-D's factory calibration populates fx/fy/cx/cy/width/height correctly in a bench-captured dataset (the new dims-consistency warning must be silent on real data). Recalibrate if not. | Footprint math, check-mode coverage, and ODM priors all consume these |
| D2 | **Bench-capture dataset** (TP-002 F2 alternative): one real recorder capture over a walkable area — validates recorder→pipeline on real imagery before flying. | De-risks the field day |
| D3 | **Orin companion prep**: rebuild `docker/Dockerfile.orin` (now preinstalls shapely/Pillow/PyYAML); **pre-pull `opendronemap/odm:3.5.4` (arm64, multi-GB) on the Orin** before leaving connectivity; confirm `/data` external volume mounted with ≥500 MB free. | TS-10/TS-11 and the on-site check depend on it |
| D4 | **Reference ground station**: designate the x86_64 + Docker machine that STK-1(c)'s ≤2 h clock is measured on; pre-pull the ODM image there too. | STK-1(c) is bound to "reference GS hardware" |
| D5 | **Field site**: real survey polygon (REF-RECT-like, ~1 ha), 40 m AGL legal/clear; GSD ≤ 5 cm at 40 m with the OAK-D per TP-002 params — verify with D2 imagery before the demo. | STK-1(b) |

## E — Known-limitation follow-ups (post-capability, optional)

- Hole-accurate orthophoto footprint (mask→polygon) to replace the v1
  bounding-box in full-mode QA — refine after A2/A4 real outputs exist.
- Projected-CRS (UTM) orthophoto support if not already forced by A2.
- FE-1 RTK path (sub-cm) — separate capability-loop action, already recorded
  in DES-004.

## Sequencing

```
merge PR #26 ──▶ A1 ──▶ A2 (+CRS fix?) ──▶ A3/A4 (Orin, needs D1–D3) ──▶ A5
             └─▶ B1, B2 (parallel)      C1 (parallel, after merge)
D1–D5 anytime before A3/A4 and C2.
Capability complete = A5 + B1 + C1–C3 done ──▶ designer flips CAP-001 status.
```
