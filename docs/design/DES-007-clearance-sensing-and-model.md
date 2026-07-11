# DES-007 — Clearance Sensing, Detection Model & Data Plan

**Status:** Draft (designer-authored, owner review on CAP-002 PR)
**Capability:** [CAP-002](../capabilities/CAP-002-premow-clearance.md)
**Safety-critical:** partially — the confirmation-maneuver slice (CLR-10) touches
`src/navigation`/`src/autonomy` flight behavior; the model/data work is not.

## Summary

Dedicated plan for the clearance sensing and detection-model work: the
detection **range/GSD analysis** (including the effective-resolution penalty
of the deployed 512×512 detector input), a **grass-occlusion model** that
divides the problem honestly between the RGB daytime final check and the
**mandatory thermal dawn sweep**, the **camera-system trade** (v1 = single
fixed camera positioned by craft maneuver; gimbal + zoom recorded as the
target evolution), and the **data collection & labeling program** that puts
the customer's actual domain — grassland at mow height, regional wildlife,
graded occlusion — into the training and evaluation sets.

## Requirements addressed

| UID | How |
|---|---|
| CLR-2 | RGB operating points, range bins, occlusion grades O0–O2, eval gates |
| CLR-9 | Dataset provenance/licensing rules, labeling protocol, collection campaigns |
| CLR-10 | Confirmation maneuver: v1 "zoom" via descend + native-resolution crop |
| CLR-11 | Thermal sensor selection criteria, integration architecture |
| CLR-12 | Dawn-sweep detection approach (classical baseline gate → fine-tuned model) |
| CLR-8 | Supplies the per-profile GSD/altitude parameters the coverage generator consumes |

---

## 1. RGB operating-point and range analysis

### 1.1 The effective-GSD problem

The OAK-D color stream is 1920×1080 @ 30 Hz, HFOV ≈ 69°; ground swath ≈
1.37 × altitude. **Native** GSD is swath/1920, but `rfdetr_node` resizes the
full frame to its 512×512 engine input, so the **effective** GSD the model
sees is ~3.75× worse. That difference decides whether a fawn is detectable:

| Altitude | Swath | Native GSD | Effective GSD (512 full-frame) |
|---|---|---|---|
| 15 m | 20.6 m | 1.1 cm/px | 4.0 cm/px |
| 20 m | 27.5 m | 1.4 cm/px | 5.4 cm/px |
| 30 m | 41.2 m | 2.1 cm/px | 8.0 cm/px |
| 40 m | 55.0 m | 2.9 cm/px | 10.7 cm/px |

Target pixel footprints at **4 cm/px effective** (the CLR-2 operating
floor): adult person ≈ 12–40 px (prone–standing oblique), child ≈ 8–20 px,
sheep/dog ≈ 15–25 px, curled fawn (~0.4 m) ≈ 10 px, hare (~0.25 m) ≈ 6 px.
Consequence: **CLR-2's class floor is fawn-size (~0.4 m) and larger**; hare
and smaller are best-effort, not required. RF-DETR's small-object strength
(the reason it was selected) is what makes the 8–15 px band workable at all.

### 1.2 Candidate operating points (selection is a WP-A experiment, not an assumption)

| Option | Altitude | How ≤4 cm/px is met | Inference cost | Coverage rate* | Risk |
|---|---|---|---|---|---|
| OP-1 low full-frame | 15 m | native full-frame → 512 | 1× (23 ms, meets COMP-3) | ~1.7 min/ha | wildlife disturbance, more battery/turns, wind margin at low AGL |
| OP-2 tiled at altitude | 25–30 m | 2×2 tiling (960×540 tiles → 512) → 4.0–4.8 cm/px effective | 4× ≈ 92 ms + stitch (SAHI-style; over the COMP-3 margin — acceptable: clearance detection is not the E2E-2 avoidance path) | ~0.9 min/ha | tile-seam duplicates (tracker must merge), latency headroom gone |
| OP-3 higher-res export | 25 m | re-export engine at ≥864 input | 1× but ~2–3× slower per frame; new engine validation | ~1.1 min/ha | Orin thermal/compute envelope alongside cuVSLAM unproven |

\* 20% side overlap, 6 m/s, straight-line; turns add ~20%. Motion blur at
6 m/s, 1/500 s exposure ≈ 1.2 cm ≈ 1 px native — acceptable.

**Decision D1:** WP-A task A3 evaluates OP-1 vs OP-2 on the range-binned
eval set (§5.4); OP-1 is the baseline (no new inference machinery), OP-2 is
adopted only if disturbance/endurance data from the first field campaign
argues for altitude. OP-3 is fallback if neither meets CLR-2. The selected
operating point is written back here and consumed by CLR-8's coverage
parameterization.

## 2. Occlusion model — what RGB can and cannot claim

Vegetation occlusion grades, annotated per bounding box (§5.3):

| Grade | Definition | Day RGB claim | Dawn thermal claim |
|---|---|---|---|
| O0 | fully visible | detect (CLR-2) | detect |
| O1 | ≤30% occluded | detect (CLR-2) | detect |
| O2 | 30–70% occluded | detect (CLR-2, reduced-recall bin tracked in eval) | detect |
| O3 | >70% / fully concealed in canopy | **out of RGB scope** | detect via thermal contrast (CLR-12); residual risk under full dense canopy is stated in the report |

This split is the reason the thermal dawn sweep is mandatory: the classic
mower casualty (roe-deer fawn bedded in tall grass) is O3 — invisible to
RGB at any resolution. Established fawn-rescue drone practice
(dawn thermal sweeps before mowing) validates the approach. Every CLR-2
eval metric is reported **binned by (range × occlusion grade × class)**;
a single aggregate recall number hides exactly the failures that matter.

## 3. Camera system trade

### 3.1 v1: single fixed camera, positioned by craft movement (Decision D2)

The existing fixed OAK-D flies the coverage pass; "localize then zoom" is
implemented **without new hardware** as the confirmation maneuver (CLR-10):

1. Coverage pass detects a candidate finding below the confirmation
   confidence threshold (or in the 6–12 px marginal band).
2. Autonomy inserts a confirmation waypoint: descend to ~10 m over the
   finding's geolocation (at 10 m: native 0.7 cm/px; a 512×512 **native
   crop** centered on the target covers 3.7 m — digital zoom with no
   resize loss, ~5× better than survey effective GSD).
3. Re-image (hover or single orbit, 2–3 s), re-run inference on the native
   crop, confirm → finding (evidence clip captured at close range, better
   for the farmer too) or dismiss → logged as dismissed candidate.
4. Budget: ~45–60 s per confirmation including transit; capped by a mission
   parameter (default 10/mission); over budget → remaining candidates are
   reported *unconfirmed* rather than dropped.

Safety-critical slice: the maneuver is a normal waypoint insertion through
the existing `/trajectory` → DES-006 bridge path (PX4 flies it; no new
control authority), but altitude-floor validation and the abort rules live
in `src/autonomy`/`src/navigation` → `safety_critical: true` on that WP.

### 3.2 Target evolution: gimbal + zoom (recorded as FE, not v1)

| Aspect | Fixed cam + maneuver (v1) | Gimbal + optical zoom (target) |
|---|---|---|
| Confirmation time | 45–60 s each (descend/return) | ~5–10 s (slew + zoom from survey altitude) |
| Wildlife disturbance | descends to 10 m | stand-off confirmation |
| Evidence quality | good (close range) | good (optical) |
| Hardware | none | gimbal + zoom cam (Siyi ZR10/ZR30-class), MAVLink gimbal protocol v2, mass/power budget |
| Integration risk | none | new driver, gimbal control in autonomy, pointing/geolocation math |

**Adoption trigger (FE-1):** field data shows confirmation time >20% of
mission time, or unconfirmed-candidate rates stay high, or evidence quality
draws farmer complaints. Until then the money stays in the data program.

## 4. Thermal sensing — the dawn sweep (Decisions D3–D5)

**Why dawn, separately from the day flight:** overnight-cooled ground vs.
~38 °C body gives maximum thermal contrast; animals (and fawns) are bedded
and stationary; partial canopy gaps are enough for a warm blob to read.
The daytime RGB flight remains the **final safety check** shortly before
mowing (sun-heated ground kills thermal contrast by mid-morning, and
things move in during the day).

### 4.1 Sensor selection (D3 — candidates; final pick needs owner/procurement sign-off)

| Candidate | Res | Notes |
|---|---|---|
| FLIR Boson 640 (LWIR, 50° lens) | 640×512 | radiometric option, UVC/USB integration on Orin, the default candidate |
| FLIR Hadron 640R | 640×512 + 64MP RGB | co-boresighted RGB — enables cross-modal label transfer (§5.2), heavier integration |
| Lower-cost 384-class cores | 384×288 | halves range for the same lens — likely fails fawn-size at useful altitude |

Geometry (Boson 640, 50° HFOV, swath ≈ 0.93 × altitude): at 20 m → 2.9 cm/px,
fawn ≈ 14 px; at 30 m → 4.4 cm/px, fawn ≈ 9 px. **Dawn-sweep operating
point: 20–25 m AGL, native-resolution inference** (640×512 needs no
destructive resize). Coverage ~1.2 min/ha — a 5 ha field ≈ 7 min.

### 4.2 Detection approach (D4)

Stage gate: **classical baseline first** — radiometric/contrast threshold +
blob extraction + altitude-derived size gating may already meet CLR-12 on
dawn imagery (it is close to what manual fawn-rescue pilots do by eye). The
fine-tuned thermal model (RF-DETR on single-channel input, or the classical
detector feeding the tracker) is built only if the baseline's false-finding
rate or occluded-recall fails the eval set. Both paths publish standard
`/detections`, so the tracker/recorder chain is modality-agnostic.

### 4.3 Integration (D5)

Thermal camera as an external ROS2 image source (`/thermal/image_raw` +
camera info), registered in `EXTERNAL_SYSTEMS`; a `thermal_detector_node`
in `src/perception` (baseline or model inference); time sync against pose
reuses the DES-004 approach; `clearance_recorder_node` records thermal
evidence clips (frames are small; clips render with a colormap for the
phone report). The mission profile (`dawn_thermal_sweep` vs
`day_rgb_check`, CLR-1) selects the active detection chain and the CLR-8
coverage parameters.

## 5. Data collection & labeling program (CLR-9)

### 5.1 Collection campaigns

| Campaign | What | Ground truth |
|---|---|---|
| R1 — RGB staged | Decoy/surrogate targets (person mannequins incl. child-scale, taxidermy/3D-printed animal decoys) placed in graded grass (O0–O3 staged by grass height), flown at 10/15/20/30/40 m, nadir + 30–45° oblique, morning/noon/evening sun, per-season repeats | decoy positions staked + RTK-surveyed → range and geolocation auto-derived per frame from pose |
| R2 — RGB opportunistic | Live livestock over partner farms; real machinery/vehicles; incidental wildlife | human-reviewed |
| R3 — hard negatives | molehills, rocks, hay bales, fence posts, troughs, shadows, plastic waste | negative-only frames |
| T1 — thermal dawn staged | **Heated decoys** (body-temperature heat pads — standard fawn-rescue training practice) in graded grass incl. fully concealed O3 placements; flown 15/20/25/30 m at dawn, repeated across ground temperatures/dew conditions | staked + surveyed, logger records decoy surface temp |
| T2 — thermal opportunistic | Dawn flights over fields with known livestock/wildlife presence | human-reviewed |

Collection flights reuse the **CAP-001 survey recorder** (MAP-2: synced
frame + pose capture) — dataset missions are ordinary recorded coverage
flights; no new capture tooling. Every campaign writes a provenance
manifest entry (owned IP).

### 5.2 Public/open datasets (licensing matrix is WP-A task A1)

RGB aerial: as listed in the implementation plan (VisDrone/SDD academic →
prototyping only; WAID, Roboflow Universe livestock/deer sets → verify;
synthetic Isaac Sim/Unity → owned). Thermal aerial is scarcer: BIRDSAI
(aerial TIR wildlife — academic, verify), FLIR ADAS + KAIST (ground-view,
pretraining texture only). Expectation to plan around: **the thermal
training/eval backbone is our own T1/T2 campaigns**, not public data. If
the Hadron (co-boresighted RGB+thermal) is selected, RGB-frame labels
transfer to thermal frames nearly free — this weighs in D3.

### 5.3 Labeling protocol

- CVAT, dual annotation + expert adjudication, quality thresholds per
  [perception_finetuning_plan.md](../architecture/perception_finetuning_plan.md)
  (IoU ≥0.9, class ≥99%, exhaustive).
- Per-box attributes: class, **occlusion grade O0–O3**, truncation,
  difficulty; range bin auto-derived from pose + staked position.
- ML-assisted pre-labeling with the deployed COCO model + human correction;
  active-learning selection (uncertainty band 0.3–0.7) per the finetuning
  plan.
- Staged (R1/T1) frames auto-verify completeness: every staked decoy in the
  camera footprint must have a box, or the frame is flagged.

### 5.4 Eval sets and volume targets

Held-out eval sets (real imagery only, whole *sites* held out to prevent
leakage), stratified by (class × range bin × occlusion grade):

| Set | Contents | Gate |
|---|---|---|
| E-RGB | ≥150 instances per critical class per range bin per grade O0–O2 | CLR-2: person ≥0.95, animal ≥0.85 recall @ ≤1 FF/ha |
| E-THERM | ≥100 heated-decoy instances per range bin per grade incl. O3 | CLR-12: animal ≥0.90 recall @ ≤1 FF/ha |
| E-NEG | ≥2 h of clean-field footage (both modalities) | false-finding budget measured here |

Training volume targets (initial, revised after the first fine-tune):
≥500 instances per (critical class × occlusion grade) real, synthetic
fill for gap classes (deer, child-scale person) — synthetic never appears
in eval sets.

## 6. Fixed design decisions

| # | Decision | Choice |
|---|---|---|
| D1 | RGB operating point | OP-1 (15 m full-frame) baseline; OP-2 (tiled 25–30 m) adopted only on field evidence; selected value written back here |
| D2 | v1 camera system | Fixed OAK-D + confirmation maneuver (descend + native crop); gimbal+zoom = FE-1 with explicit adoption trigger |
| D3 | Thermal sensor | Boson-640-class default; Hadron 640R if cross-modal labeling value wins; **procurement sign-off required** |
| D4 | Thermal detection | Classical contrast/blob baseline gated first; ML only if baseline fails E-THERM |
| D5 | Thermal integration | External driver + `thermal_detector_node`; modality-agnostic `/detections` into the tracker/recorder chain |
| D6 | Eval discipline | All CLR-2/CLR-12 metrics binned by range × occlusion × class; aggregate-only numbers are not acceptance evidence |
| D7 | Class floor | Required: ≥0.4 m (fawn-size) targets; smaller (hare) best-effort, documented in the report's limitation note |

## 7. Open items

- Thermal sensor purchase (D3) — owner decision; blocks WP-G hardware tasks
  but not WP-H dataset/baseline prep (public thermal data + simulator).
- Decoy inventory for R1/T1 (child-scale mannequin, heated animal decoys).
- Partner-farm access for R2/T2 campaigns.
- Regulatory check: dawn flights and low-AGL (10 m confirmation) operation
  under local UAS rules — operator responsibility, flagged for the ConOps.

## Traceability markers expected

`Implements: CLR-10` (autonomy/navigation confirmation maneuver),
`Implements: CLR-11` (thermal integration), `Implements: CLR-12`
(thermal detection), plus the WP-A markers (`Implements: CLR-2`, `CLR-9`)
per the implementation plan.
