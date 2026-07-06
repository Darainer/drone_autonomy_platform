# RF-DETR Fine-Tuning Plan: Data Collection & Model Training

## Overview

This document defines the data collection strategy and fine-tuning process for adapting RF-DETR-Small to surveying and agricultural autonomy mission requirements. The goal is to transition from stock COCO-pretrained weights to a domain-specific model optimized for aerial detection of civilian surveying and agricultural targets.

### Relationship to Architecture

This plan implements Phases 2-3 of the model deployment strategy defined in [perception_architecture.md](perception_architecture.md):

| Phase | Model | Status |
|-------|-------|--------|
| Phase 1 | RF-DETR-Small, COCO pretrained | Current deployment |
| **Phase 2** | RF-DETR-Small + COCO + domain data | **This plan** |
| **Phase 3** | Custom RF-DETR-Small (mission-specific) | **This plan** |

---

## Target Class Taxonomy

### Primary Target Classes

| Class ID | Class Name | Description | Priority |
|----------|------------|-------------|----------|
| 0 | `tractor` | Agricultural tractors | Critical |
| 1 | `harvester` | Combine harvesters | Critical |
| 2 | `sprayer` | Crop sprayers | High |
| 3 | `vehicle_car` | Civilian cars, SUVs | Medium |
| 4 | `vehicle_truck` | Pickup trucks, cargo trucks | High |
| 5 | `livestock_cattle` | Cattle | Medium |
| 6 | `livestock_sheep` | Sheep | Medium |
| 7 | `livestock_horse` | Horses | Medium |
| 8 | `obstacle_tree` | Trees | Critical |
| 9 | `obstacle_building` | Buildings, sheds | High |
| 10 | `obstacle_fence` | Fencelines | High |
| 11 | `infra_powerline` | Power lines / towers | Critical |
| 12 | `infra_wind_turbine` | Wind turbines | High |

### Class Hierarchy

```
targets/
├── machinery/
│   ├── tractor
│   ├── harvester
│   └── sprayer
├── vehicles/
│   ├── vehicle_car
│   └── vehicle_truck
├── livestock/
│   ├── livestock_cattle
│   ├── livestock_sheep
│   └── livestock_horse
├── obstacles/
│   ├── obstacle_tree
│   ├── obstacle_building
│   └── obstacle_fence
└── infrastructure/
    ├── infra_powerline
    └── infra_wind_turbine
```

---

## Data Collection Strategy

### Data Sources

| Source | Type | Volume Target | Advantages | Challenges |
|--------|------|---------------|------------|------------|
| **Synthetic (Unity/Unreal)** | Generated | 50,000 images | Unlimited variety, perfect labels | Domain gap |
| **Flight test footage** | Real | 10,000 images | Real sensor characteristics | Limited scenarios |
| **Public datasets** | Real | 20,000 images | Pre-annotated, diverse | May not match use case |
| **Partner/co-op data** | Real | 15,000 images | Mission-relevant (agricultural co-ops, survey firms) | Access restrictions |

**Total Target: 95,000+ annotated images**

### Phase 2: Foundation Dataset

#### 2.1 Public Dataset Integration

| Dataset | Classes | Images | Relevance | License |
|---------|---------|--------|-----------|---------|
| **VisDrone2021** | Vehicles, pedestrians | 10,209 | High (UAV perspective) | Academic |
| **DOTA v2.0** | Vehicles, ships, planes | 11,268 | High (aerial) | Academic |
| **xView** | 60 classes | 1,000,000 chips | High (satellite/aerial) | DIUx |
| **VEDAI** | Vehicles | 1,210 | Medium (small vehicles) | Academic |
| **COWC** | Cars | 32,716 | Medium (counting) | Academic |

**Preprocessing Required:**
- Remap class labels to target taxonomy
- Filter irrelevant classes
- Resize/crop to 640×640 with context
- Convert annotations to COCO JSON format

#### 2.1.1 Dataset Licensing for Commercial Use

> ⚠️ **Critical**: Academic-licensed datasets (VisDrone, DOTA, VEDAI, COWC) are suitable for research prototyping only. Commercial product deployment requires either separate licensing agreements or use of permissively-licensed alternatives.

**Commercially-Viable Open Datasets (AWS Open Data / Permissive Licenses):**

| Dataset | License | Images | Classes | Commercial Use | Notes |
|---------|---------|--------|---------|----------------|-------|
| **SpaceNet** | CC-BY-SA 4.0 | 100K+ | Buildings, roads, vehicles | ✅ Yes (with attribution) | High-res satellite, multiple challenges |
| **LADI** | Open Government | 50K+ | Infrastructure, damage, vehicles | ✅ Yes | UAV perspective, disaster response |
| **DeepForest** | MIT | 5K+ | Trees, birds | ✅ Yes | Limited class diversity |
| **OpenAerialMap** | Varies (mostly CC-BY) | 100K+ | Unlabeled imagery | ✅ Check per-image | Requires annotation |
| **RarePlanes** | Open | 50K+ | Aircraft | ✅ Yes | Real + synthetic satellite |

**Datasets Requiring License Review:**

| Dataset | License | Commercial Status | Action Required |
|---------|---------|-------------------|-----------------|
| **xView** | DIUx Terms | ⚠️ Review required | Contact NGA/DIUx for commercial terms |
| **VisDrone** | Academic | ❌ Research only | Seek commercial license or replace |
| **DOTA** | Academic | ❌ Research only | Seek commercial license or replace |

**Recommended Dataset Strategy:**

1. **Phase 2 Development**: Use VisDrone/DOTA for rapid prototyping and architecture validation
2. **Phase 2 Production**: Transition to SpaceNet + LADI + xView (after license confirmation)
3. **Phase 3**: Prioritize internal data collection to eliminate external licensing dependencies

**Roboflow Universe Datasets (CC-BY/Apache licensed):**

- Aerial Maritime Drone Dataset (~500 images) - boats, cars, docks, jetskis
- Drone Aerial Highway Footage (~50 images) - vehicles from UAV perspective
- Various aerial detection datasets - verify individual licenses before use

**Synthetic Data Advantage**: Data generated via Isaac Sim/Unity is owned IP with no external licensing constraints - prioritize synthetic data expansion to reduce dependency on external datasets.

#### 2.2 Synthetic Data Generation

**Simulation Environment: NVIDIA Isaac Sim / Unity**

| Parameter | Specification |
|-----------|---------------|
| Engine | Unity 2022 LTS + Perception package |
| Assets | Sketchfab agricultural models, Turbosquid vehicles |
| Terrains | Farmland, orchard, rangeland, coastal, snow |
| Weather | Clear, overcast, rain, fog, dust |
| Lighting | Dawn, day, dusk, low-light operations |
| Camera | 1080p, 60° FOV (matching target sensor) |

**Synthetic Data Parameters:**

```yaml
# synthetic_generation_config.yaml
camera:
  altitude_range: [50, 500]  # meters AGL
  pitch_range: [-90, -30]    # degrees (nadir to oblique)
  speed_range: [0, 30]       # m/s platform motion
  
targets:
  count_range: [1, 15]       # objects per frame
  occlusion_levels: [0, 0.3, 0.5, 0.7]
  spacing: "realistic"       # field/yard layout dispersion
  
augmentation:
  motion_blur: true
  sensor_noise: true
  compression_artifacts: true
  
output:
  format: "coco"
  images_per_scenario: 1000
  total_scenarios: 50
```

**Domain Randomization Checklist:**
- [ ] Vehicle/machinery paint schemes and liveries
- [ ] Personnel poses and clothing (farm/field workers)
- [ ] Shadow angles (sun position)
- [ ] Ground texture variation
- [ ] Atmospheric haze/dust
- [ ] Sensor noise profiles

### Phase 3: Mission-Specific Dataset

#### 3.1 Flight Test Data Collection

**Collection Protocol:**

| Flight Type | Purpose | Hours | Expected Images |
|-------------|---------|-------|-----------------|
| Controlled target | Known targets, varied conditions | 20 | 5,000 |
| Operational exercise | Realistic scenarios | 40 | 8,000 |
| Edge case hunting | Failure mode collection | 10 | 2,000 |

**Target Placement Requirements:**
- Minimum 3 examples per class per terrain type
- Multiple aspect angles (0°, 45°, 90°, 135°, 180°)
- Range variation (near: 50m, mid: 150m, far: 400m)
- Partial occlusion scenarios (foliage, structures, other machinery)

#### 3.2 Hard Negative Mining

Collect examples of:
- Agricultural machinery lookalikes (construction equipment, generic trailers)
- Natural objects resembling targets (rock formations, vegetation patterns)
- Parked/idle machinery and equipment stored under tarps or covers
- Partial/damaged equipment

---

## Annotation Requirements

### Annotation Specification

| Field | Format | Required |
|-------|--------|----------|
| Bounding box | COCO [x, y, width, height] | Yes |
| Class label | Integer (0-16) | Yes |
| Occlusion level | Float (0.0-1.0) | Yes |
| Truncation | Boolean | Yes |
| Difficulty | easy/medium/hard | Yes |
| Instance ID | Unique per object | For tracking data |

### Annotation Quality Standards

| Metric | Threshold | Verification |
|--------|-----------|--------------|
| IoU accuracy | ≥0.9 with ground truth | Dual annotation + adjudication |
| Class accuracy | ≥99% | Expert review |
| Completeness | 100% visible objects | Exhaustive labeling protocol |
| Consistency | Cross-annotator agreement ≥95% | Regular calibration sessions |

### Annotation Workflow

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Ingest    │───▶│  Annotate   │───▶│   Review    │───▶│   Export    │
│             │    │  (CVAT/LS)  │    │   (QA)      │    │  (COCO)     │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
      │                  │                  │                  │
      ▼                  ▼                  ▼                  ▼
  Raw images      Dual annotation     Adjudication      Train/val/test
  + metadata      by 2 labelers       by expert         splits
```

### Annotation Tools

| Tool | Use Case | Notes |
|------|----------|-------|
| **CVAT** | Primary annotation | Self-hosted, supports COCO export |
| **Label Studio** | Complex scenarios | ML-assisted labeling |
| **FiftyOne** | Dataset curation | Visualization, error analysis |
| **Roboflow** | Augmentation pipeline | Preprocessing automation |

---

## Dataset Structure

### Directory Layout

```
datasets/
├── survey_targets_v1/
│   ├── annotations/
│   │   ├── train.json          # COCO format
│   │   ├── val.json
│   │   └── test.json
│   ├── images/
│   │   ├── train/
│   │   ├── val/
│   │   └── test/
│   ├── metadata/
│   │   ├── sources.csv         # Image provenance
│   │   ├── class_distribution.json
│   │   └── quality_report.json
│   └── config.yaml             # Dataset configuration
├── synthetic_v1/
│   └── ...
└── flight_test_v1/
    └── ...
```

### Dataset Splits

| Split | Percentage | Purpose | Stratification |
|-------|------------|---------|----------------|
| Train | 70% | Model training | By class, source, difficulty |
| Validation | 15% | Hyperparameter tuning | Same stratification |
| Test | 15% | Final evaluation | Held-out, no peeking |

**Stratification Requirements:**
- Each split contains all classes
- Proportional representation by data source
- Difficulty levels balanced
- Terrain types distributed

### COCO Annotation Format

```json
{
  "info": {
    "description": "Survey Target Detection Dataset v1",
    "version": "1.0",
    "year": 2026,
    "contributor": "Drone Autonomy Platform",
    "date_created": "2026-01-20"
  },
  "licenses": [],
  "categories": [
    {"id": 0, "name": "tractor", "supercategory": "machinery"},
    {"id": 1, "name": "harvester", "supercategory": "machinery"},
    {"id": 5, "name": "livestock_cattle", "supercategory": "livestock"}
  ],
  "images": [
    {
      "id": 1,
      "file_name": "train/00001.jpg",
      "width": 1920,
      "height": 1080,
      "metadata": {
        "source": "synthetic",
        "altitude_m": 150,
        "terrain": "desert",
        "weather": "clear"
      }
    }
  ],
  "annotations": [
    {
      "id": 1,
      "image_id": 1,
      "category_id": 5,
      "bbox": [100, 200, 50, 30],
      "area": 1500,
      "iscrowd": 0,
      "attributes": {
        "occlusion": 0.1,
        "truncated": false,
        "difficulty": "easy"
      }
    }
  ]
}
```

---

## Data Augmentation Pipeline

### Training-Time Augmentations

| Augmentation | Probability | Parameters | Rationale |
|--------------|-------------|------------|-----------|
| Horizontal flip | 0.5 | - | Symmetry invariance |
| Random crop | 0.5 | 0.8-1.0 scale | Scale invariance |
| Color jitter | 0.3 | brightness=0.2, contrast=0.2 | Lighting robustness |
| Motion blur | 0.2 | kernel=5-15 | Platform motion |
| Gaussian noise | 0.2 | σ=0.01-0.05 | Sensor noise |
| JPEG compression | 0.3 | quality=50-95 | Transmission artifacts |
| Cutout/GridMask | 0.2 | 1-3 patches | Occlusion robustness |
| Mosaic | 0.5 | 4-image | Context diversity |
| MixUp | 0.1 | α=0.5 | Regularization |

### Augmentation Configuration

```yaml
# augmentation_config.yaml
train:
  mosaic: 0.5
  mixup: 0.1
  hsv_h: 0.015
  hsv_s: 0.7
  hsv_v: 0.4
  degrees: 10
  translate: 0.1
  scale: 0.5
  shear: 2.0
  perspective: 0.0001
  flipud: 0.0        # No vertical flip (unnatural for aerial)
  fliplr: 0.5
  mosaic_border: [-320, -320]
  
custom:
  motion_blur:
    prob: 0.2
    kernel_range: [5, 15]
  sensor_noise:
    prob: 0.2
    sigma_range: [0.01, 0.05]
  atmospheric_haze:
    prob: 0.15
    intensity_range: [0.1, 0.4]
```

---

## Fine-Tuning Process

### Training Infrastructure

| Component | Specification |
|-----------|---------------|
| GPU | NVIDIA A100 80GB (training) |
| Framework | `rfdetr` (Roboflow) + PyTorch 2.x |
| Experiment tracking | Weights & Biases |
| Model registry | MLflow |

### Phase 2 Training: Foundation Fine-Tune

**Objective:** Adapt RF-DETR-Small from COCO weights to the aerial survey/agriculture domain.

RF-DETR fine-tunes via the Roboflow `rfdetr` package (the same package used for export in
`scripts/export_rfdetr_onnx.py`), against a COCO-format dataset. Keep this snippet as a
starting point — check the [rfdetr docs](https://github.com/roboflow/rf-detr) for the full
set of supported training arguments before relying on any option not shown here.

```python
# phase2_finetune.py
from rfdetr import RFDETRSmall
import wandb

wandb.init(project="survey-rfdetr", name="phase2-foundation")

# Load COCO-pretrained RF-DETR-Small
model = RFDETRSmall()

# Fine-tune on the Phase 2 dataset (COCO-format directory, see Dataset Structure below)
model.train(
    dataset_dir="datasets/survey_targets_v1",
    epochs=50,
    batch_size=16,
    lr=1e-4,              # lower LR for fine-tuning from COCO weights
    output_dir="runs/phase2_foundation",
)
```

### Phase 3 Training: Mission-Specific

**Objective:** Specialize for operational target classes.

```python
# phase3_finetune.py
from rfdetr import RFDETRSmall

# Start from the Phase 2 fine-tuned checkpoint
model = RFDETRSmall(pretrain_weights="runs/phase2_foundation/best.pth")

# Fine-tune on mission-specific data
model.train(
    dataset_dir="datasets/flight_test_v1",
    epochs=50,
    batch_size=16,
    lr=1e-5,               # lower LR for the smaller, mission-specific dataset
    output_dir="runs/phase3_mission",
)
```

### Hyperparameter Search

| Parameter | Search Range | Method |
|-----------|--------------|--------|
| Learning rate | [1e-5, 1e-3] | Log uniform |
| Batch size | [8, 16, 32] | Grid |

```python
# hyperparameter_sweep.py
import wandb

sweep_config = {
    "method": "bayes",
    "metric": {"name": "mAP50-95", "goal": "maximize"},
    "parameters": {
        "lr": {"distribution": "log_uniform_values", "min": 1e-5, "max": 1e-3},
        "batch_size": {"values": [8, 16, 32]},
    }
}

sweep_id = wandb.sweep(sweep_config, project="survey-rfdetr")
wandb.agent(sweep_id, function=train_with_config, count=30)
```

---

## Evaluation Metrics

### Primary Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| **mAP@0.5** | ≥60% | COCO evaluation |
| **mAP@0.5:0.95** | ≥50% | COCO evaluation |
| **Critical class AP** | ≥70% | Per-class (tractor, harvester, obstacle_tree, infra_powerline) |
| **Small object AP** | ≥40% | Objects <32×32 pixels |
| **Inference latency** | ≤25ms | TensorRT on Orin |

### Operational Metrics

| Metric | Target | Description |
|--------|--------|-------------|
| **False positive rate** | <5% | Incorrect detections per frame |
| **Miss rate (critical)** | <2% | Missed critical targets |
| **Track stability** | >90% | Consistent ID over 30 frames |
| **Detection range** | 400m | mAP ≥50% at max range |

### Evaluation Protocol

```python
# evaluate_model.py
from rfdetr import RFDETRSmall
from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval

# Load fine-tuned model
model = RFDETRSmall(pretrain_weights="models/phase3_best.pth")

# Run validation (consult the rfdetr docs for the exact eval entrypoint/args)
metrics = model.val(
    dataset_dir="datasets/survey_targets_v1",
    split="test",
)

# Per-class analysis
print("Per-class AP@0.5:")
for i, ap in enumerate(metrics.box.ap50):
    print(f"  {class_names[i]}: {ap:.3f}")

# Critical class check (see Target Class Taxonomy: tractor, harvester, obstacle_tree, infra_powerline)
critical_classes = [0, 1, 8, 11]  # tractor, harvester, obstacle_tree, infra_powerline
critical_ap = [metrics.box.ap50[i] for i in critical_classes]
assert min(critical_ap) >= 0.70, "Critical class AP below threshold"
```

### Confusion Matrix Analysis

Generate and review:
- Inter-class confusion (vehicle_car vs. vehicle_truck, tractor vs. harvester)
- False positive sources (background objects)
- False negative patterns (occlusion, distance, angle)

---

## Model Deployment Pipeline

### Export and Validation

```bash
#!/bin/bash
# export_and_validate.sh
# Same two-stage flow as scripts/export_rfdetr_onnx.py + scripts/build_tensorrt_engine.sh,
# but pointed at a fine-tuned Phase 3 checkpoint instead of stock COCO weights.
# export_rfdetr_onnx.py currently loads stock RFDETRSmall() weights only — loading a
# fine-tuned checkpoint requires extending it to pass pretrain_weights= through to
# RFDETRSmall(), per the rfdetr docs (https://github.com/roboflow/rf-detr).

ONNX_PATH="models/phase3_best.onnx"

# 1. Export the fine-tuned checkpoint to ONNX (workstation GPU)
python scripts/export_rfdetr_onnx.py --output "${ONNX_PATH}"

# 2. Build the TensorRT engine on the Jetson
scp "${ONNX_PATH}" jetson@orin-dev:/home/dev/models/phase3_best.onnx
ssh jetson@orin-dev "cd /path/to/repo && scripts/build_tensorrt_engine.sh /home/dev/models/phase3_best.onnx"
```

### Deployment Checklist

- [ ] mAP@0.5 ≥60% on test set
- [ ] Critical class AP ≥70%
- [ ] Inference latency ≤25ms on Orin
- [ ] No regression on baseline scenarios
- [ ] Edge case review completed
- [ ] Confusion matrix reviewed
- [ ] Model card documented
- [ ] Versioned in model registry

### Model Versioning

| Version | Date | Dataset | mAP@0.5 | Notes |
|---------|------|---------|---------|-------|
| v0.1 | 2026-01 | RF-DETR-Small (COCO pretrained) | 45% | Baseline pretrained |
| v1.0 | 2026-Q2 | Phase 2 dataset | 55% | Foundation fine-tune |
| v2.0 | 2026-Q3 | Phase 3 dataset | 65% | Mission-specific |

---

## Continuous Improvement

### Feedback Loop

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      Continuous Improvement Loop                         │
│                                                                          │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐          │
│  │ Deploy   │───▶│ Monitor  │───▶│ Collect  │───▶│ Retrain  │──┐       │
│  │          │    │ (Ops)    │    │ (Errors) │    │          │  │       │
│  └──────────┘    └──────────┘    └──────────┘    └──────────┘  │       │
│       ▲                                                         │       │
│       └─────────────────────────────────────────────────────────┘       │
└─────────────────────────────────────────────────────────────────────────┘
```

### Operational Data Collection

During missions:
- Log all detections with confidence scores
- Flag low-confidence detections for review
- Capture frames where operator overrides model
- Record miss reports from operators

### Retraining Triggers

| Trigger | Threshold | Action |
|---------|-----------|--------|
| New target class needed | N/A | Collect data, retrain |
| mAP degradation | >5% drop | Investigate, retrain |
| False positive spike | >10% increase | Hard negative mining |
| New operating environment | N/A | Domain adaptation |

### Active Learning Pipeline

```python
# active_learning.py

def select_samples_for_annotation(predictions, budget=1000):
    """Select most informative samples for human annotation."""
    
    samples = []
    
    for pred in predictions:
        # Uncertainty sampling: low confidence detections
        if 0.3 < pred.confidence < 0.7:
            samples.append((pred.image_id, "uncertainty"))
        
        # Disagreement: multiple overlapping boxes
        if has_overlapping_detections(pred):
            samples.append((pred.image_id, "disagreement"))
        
        # Novelty: unusual feature embeddings
        if is_out_of_distribution(pred):
            samples.append((pred.image_id, "novelty"))
    
    # Prioritize and return top samples
    return prioritize(samples, budget)
```

---

## Timeline

| Milestone | Target Date | Deliverable |
|-----------|-------------|-------------|
| Dataset infrastructure | 2026-02-15 | Annotation pipeline operational |
| Phase 2 dataset complete | 2026-03-31 | 50K+ annotated images |
| Phase 2 model trained | 2026-04-30 | mAP ≥55% |
| Phase 2 deployed | 2026-05-15 | TensorRT engine on Orin |
| Flight test data collection | 2026-06-30 | 10K real-world images |
| Phase 3 dataset complete | 2026-08-31 | 95K+ annotated images |
| Phase 3 model trained | 2026-09-30 | mAP ≥65% |
| Phase 3 deployed | 2026-10-15 | Production model |
| Continuous improvement | Ongoing | Quarterly retraining |

---

## References

- [perception_architecture.md](perception_architecture.md) - Pipeline architecture
- [latency_requirements.md](latency_requirements.md) - Latency budgets
- [VisDrone Dataset](https://github.com/VisDrone/VisDrone-Dataset)
- [DOTA Dataset](https://captain-whu.github.io/DOTA/)
- [xView Dataset](http://xviewdataset.org/)
- [RF-DETR (Roboflow) GitHub](https://github.com/roboflow/rf-detr) - training/export code and docs
