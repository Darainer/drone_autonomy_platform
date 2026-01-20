# RT-DETR Fine-Tuning Plan: Data Collection & Model Training

## Overview

This document defines the data collection strategy and fine-tuning process for adapting RT-DETR-L to ISR and target acquisition mission requirements. The goal is to transition from SyntheticaDETR pretrained weights to a domain-specific model optimized for aerial detection of military-relevant targets.

### Relationship to Architecture

This plan implements Phases 2-3 of the model deployment strategy defined in [perception_architecture.md](perception_architecture.md):

| Phase | Model | Status |
|-------|-------|--------|
| Phase 1 | SyntheticaDETR (pretrained) | Current deployment |
| **Phase 2** | RT-DETR-L + COCO + domain data | **This plan** |
| **Phase 3** | Custom RT-DETR-L (mission-specific) | **This plan** |

---

## Target Class Taxonomy

### Primary Target Classes

| Class ID | Class Name | Description | Priority |
|----------|------------|-------------|----------|
| 0 | `person_standing` | Individual standing/walking | High |
| 1 | `person_prone` | Individual prone/crawling | High |
| 2 | `person_group` | Group of 3+ individuals | High |
| 3 | `vehicle_car` | Civilian cars, SUVs | Medium |
| 4 | `vehicle_truck` | Pickup trucks, cargo trucks | High |
| 5 | `vehicle_technical` | Armed pickup trucks | Critical |
| 6 | `vehicle_apc` | Armored personnel carriers | Critical |
| 7 | `vehicle_tank` | Main battle tanks | Critical |
| 8 | `vehicle_artillery` | Towed/self-propelled artillery | Critical |
| 9 | `aircraft_rotary` | Helicopters | High |
| 10 | `aircraft_fixed` | Fixed-wing aircraft | High |
| 11 | `aircraft_uav` | Small UAVs/drones | Medium |
| 12 | `watercraft_patrol` | Patrol boats | Medium |
| 13 | `watercraft_rib` | Rigid inflatable boats | Medium |
| 14 | `infrastructure_radar` | Radar installations | High |
| 15 | `infrastructure_sam` | SAM sites | Critical |
| 16 | `infrastructure_bunker` | Bunkers, fortifications | Medium |

### Class Hierarchy

```
targets/
├── personnel/
│   ├── person_standing
│   ├── person_prone
│   └── person_group
├── ground_vehicles/
│   ├── civilian/
│   │   ├── vehicle_car
│   │   └── vehicle_truck
│   └── military/
│       ├── vehicle_technical
│       ├── vehicle_apc
│       ├── vehicle_tank
│       └── vehicle_artillery
├── aircraft/
│   ├── aircraft_rotary
│   ├── aircraft_fixed
│   └── aircraft_uav
├── watercraft/
│   ├── watercraft_patrol
│   └── watercraft_rib
└── infrastructure/
    ├── infrastructure_radar
    ├── infrastructure_sam
    └── infrastructure_bunker
```

---

## Data Collection Strategy

### Data Sources

| Source | Type | Volume Target | Advantages | Challenges |
|--------|------|---------------|------------|------------|
| **Synthetic (Unity/Unreal)** | Generated | 50,000 images | Unlimited variety, perfect labels | Domain gap |
| **Flight test footage** | Real | 10,000 images | Real sensor characteristics | Limited scenarios |
| **Public datasets** | Real | 20,000 images | Pre-annotated, diverse | May not match use case |
| **Partner/DoD data** | Real | 15,000 images | Mission-relevant | Access restrictions |

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

#### 2.2 Synthetic Data Generation

**Simulation Environment: NVIDIA Isaac Sim / Unity**

| Parameter | Specification |
|-----------|---------------|
| Engine | Unity 2022 LTS + Perception package |
| Assets | Sketchfab military models, Turbosquid vehicles |
| Terrains | Desert, urban, forest, coastal, snow |
| Weather | Clear, overcast, rain, fog, dust |
| Lighting | Dawn, day, dusk, night (NVG simulation) |
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
  spacing: "realistic"       # tactical dispersion
  
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
- [ ] Vehicle paint schemes (camo patterns, civilian colors)
- [ ] Personnel poses and clothing
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
- Partial occlusion scenarios
- Camouflage/concealment examples

#### 3.2 Hard Negative Mining

Collect examples of:
- Civilian vehicles similar to military targets
- Natural objects resembling targets (rock formations, vegetation patterns)
- Decoys and mock targets
- Partial/damaged vehicles
- Thermal vs. visual appearance discrepancies (future IR fusion)

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
├── isr_targets_v1/
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
    "description": "ISR Target Detection Dataset v1",
    "version": "1.0",
    "year": 2026,
    "contributor": "Drone Autonomy Platform",
    "date_created": "2026-01-20"
  },
  "licenses": [],
  "categories": [
    {"id": 0, "name": "person_standing", "supercategory": "personnel"},
    {"id": 1, "name": "person_prone", "supercategory": "personnel"},
    {"id": 5, "name": "vehicle_technical", "supercategory": "ground_vehicles"}
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
| Framework | Ultralytics + PyTorch 2.x |
| Distributed | DDP across 4 GPUs |
| Experiment tracking | Weights & Biases |
| Model registry | MLflow |

### Phase 2 Training: Foundation Fine-Tune

**Objective:** Adapt RT-DETR-L from COCO weights to aerial domain

```python
# phase2_finetune.py
from ultralytics import RTDETR
import wandb

wandb.init(project="isr-rtdetr", name="phase2-foundation")

# Load COCO pretrained model
model = RTDETR("rtdetr-l.pt")

# Phase 2a: Freeze backbone, train head
results = model.train(
    data="isr_targets_v1.yaml",
    epochs=50,
    imgsz=640,
    batch=32,
    device=[0, 1, 2, 3],
    workers=8,
    
    # Optimizer
    optimizer="AdamW",
    lr0=0.0001,           # Lower LR for fine-tuning
    lrf=0.01,             # Final LR ratio
    warmup_epochs=5,
    weight_decay=0.0001,
    
    # Regularization
    dropout=0.0,
    
    # Backbone freeze
    freeze=10,            # Freeze first 10 layers
    
    # Augmentation
    augment=True,
    mosaic=0.5,
    mixup=0.1,
    
    # Validation
    val=True,
    plots=True,
    
    # Checkpointing
    save=True,
    save_period=5,
)

# Phase 2b: Unfreeze and full fine-tune
model = RTDETR("runs/detect/phase2a/weights/best.pt")
results = model.train(
    data="isr_targets_v1.yaml",
    epochs=100,
    imgsz=640,
    batch=16,             # Smaller batch for full model
    device=[0, 1, 2, 3],
    
    lr0=0.00001,          # Even lower LR
    freeze=0,             # Unfreeze all
    
    resume=False,
)
```

### Phase 3 Training: Mission-Specific

**Objective:** Specialize for operational target classes

```python
# phase3_finetune.py

# Start from Phase 2 best weights
model = RTDETR("models/phase2_best.pt")

# Fine-tune on mission-specific data
results = model.train(
    data="mission_targets_v1.yaml",
    epochs=50,
    imgsz=640,
    batch=16,
    
    lr0=0.00001,
    freeze=15,            # Freeze more layers
    
    # Class weighting for imbalanced data
    cls_pw=1.5,           # Positive weight for rare classes
    
    # Hard example mining
    close_mosaic=10,      # Disable mosaic last 10 epochs
)
```

### Hyperparameter Search

| Parameter | Search Range | Method |
|-----------|--------------|--------|
| Learning rate | [1e-5, 1e-3] | Log uniform |
| Batch size | [8, 16, 32] | Grid |
| Freeze layers | [0, 5, 10, 15] | Grid |
| Weight decay | [1e-5, 1e-3] | Log uniform |
| Mosaic prob | [0.0, 0.5, 1.0] | Grid |
| Dropout | [0.0, 0.1, 0.2] | Grid |

```python
# hyperparameter_sweep.py
import wandb

sweep_config = {
    "method": "bayes",
    "metric": {"name": "metrics/mAP50-95", "goal": "maximize"},
    "parameters": {
        "lr0": {"distribution": "log_uniform_values", "min": 1e-5, "max": 1e-3},
        "batch": {"values": [8, 16, 32]},
        "freeze": {"values": [0, 5, 10, 15]},
        "weight_decay": {"distribution": "log_uniform_values", "min": 1e-5, "max": 1e-3},
    }
}

sweep_id = wandb.sweep(sweep_config, project="isr-rtdetr")
wandb.agent(sweep_id, function=train_with_config, count=30)
```

---

## Evaluation Metrics

### Primary Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| **mAP@0.5** | ≥60% | COCO evaluation |
| **mAP@0.5:0.95** | ≥50% | COCO evaluation |
| **Critical class AP** | ≥70% | Per-class (tank, APC, SAM) |
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
from ultralytics import RTDETR
from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval

# Load model
model = RTDETR("models/phase3_best.pt")

# Run validation
metrics = model.val(
    data="isr_targets_v1.yaml",
    split="test",
    imgsz=640,
    batch=1,
    device=0,
    plots=True,
    save_json=True,
)

# Per-class analysis
print("Per-class AP@0.5:")
for i, ap in enumerate(metrics.box.ap50):
    print(f"  {class_names[i]}: {ap:.3f}")

# Critical class check
critical_classes = [5, 6, 7, 8, 15]  # technical, apc, tank, artillery, sam
critical_ap = [metrics.box.ap50[i] for i in critical_classes]
assert min(critical_ap) >= 0.70, "Critical class AP below threshold"
```

### Confusion Matrix Analysis

Generate and review:
- Inter-class confusion (car vs. truck, APC vs. tank)
- False positive sources (background objects)
- False negative patterns (occlusion, distance, angle)

---

## Model Deployment Pipeline

### Export and Validation

```bash
#!/bin/bash
# export_and_validate.sh

MODEL_PATH="models/phase3_best.pt"
OUTPUT_DIR="deploy/"

# Export to ONNX
python -c "
from ultralytics import RTDETR
model = RTDETR('${MODEL_PATH}')
model.export(format='onnx', imgsz=640, opset=17, simplify=True)
"

# Convert to TensorRT on Jetson
ssh jetson@orin-dev "
cd /workspaces/isaac_ros-dev
trtexec \
    --onnx=/models/phase3_best.onnx \
    --saveEngine=/models/phase3_best.plan \
    --fp16 \
    --workspace=4096 \
    --minShapes=images:1x3x640x640 \
    --optShapes=images:1x3x640x640 \
    --maxShapes=images:1x3x640x640
"

# Validate on Jetson
ssh jetson@orin-dev "
python validate_tensorrt.py \
    --engine /models/phase3_best.plan \
    --test-images /data/test_images/ \
    --expected-latency 25
"
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
| v0.1 | 2026-01 | SyntheticaDETR | 45% | Baseline pretrained |
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
- [isr.md](use_cases/isr.md) - ISR mission requirements
- [target_acquisition.md](use_cases/target_acquisition.md) - Target acquisition requirements
- [VisDrone Dataset](https://github.com/VisDrone/VisDrone-Dataset)
- [DOTA Dataset](https://captain-whu.github.io/DOTA/)
- [xView Dataset](http://xviewdataset.org/)
- [Ultralytics Training Guide](https://docs.ultralytics.com/modes/train/)
