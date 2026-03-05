# Collaborative HD Mapping with Camera-LiDAR Fusion and Fingerprint Synchronization

## Use Case Description

Multi-platform collaborative construction and sharing of semantically-enriched high-definition maps using fused camera and LiDAR sensing. Map segments are compressed and fingerprinted for efficient transmission between aerial drones, ground vehicles, and command assets over bandwidth-constrained tactical data links.

### Operational Scenario

A heterogeneous team of assets collaboratively builds and shares situational awareness:
- **Aerial drones** provide wide-area coverage, overhead perspective, and RGB/thermal semantic classification
- **Ground vehicles/UGVs** provide high-density ground-level scans with detailed facade and obstacle semantics
- **Command post** aggregates, fuses, and distributes combined maps to all networked assets

Each platform builds local HD map segments by fusing geometric structure from LiDAR with semantic understanding from camera-based perception. Compact fingerprint hashes enable efficient delta synchronization—platforms exchange only what has changed, preserving precious tactical bandwidth while maintaining shared situational awareness across the force.

---

## Military Value and Operational Significance

### Force Multiplication Through Shared Understanding

Collaborative mapping fundamentally transforms how distributed forces operate by creating a single, coherent picture of the battlespace that updates in real-time as any platform makes new observations. When an aerial drone detects a new structure or a ground vehicle identifies a concealed fighting position, that knowledge propagates to every connected asset within seconds. This eliminates the traditional fog of war that arises when individual platforms operate with incomplete, stale, or contradictory information. A convoy approaching an urban area receives the detailed street-level mapping that a preceding UGV collected, while simultaneously benefiting from the overhead route analysis an orbiting UAV generated. The whole becomes dramatically greater than the sum of its parts.

### Bandwidth Efficiency in Contested Environments

Modern peer adversaries possess sophisticated electronic warfare capabilities that degrade, deny, and disrupt tactical communications. Traditional approaches to map sharing—transmitting full imagery or point clouds—consume bandwidth measured in megabytes and require sustained connectivity that cannot be guaranteed. The fingerprint-based synchronization protocol reduces this to kilobytes by transmitting only compact hashes during normal operations and compressed deltas when differences are detected. A 50-tile map covering 125,000 square meters can be verified with a 2 KB message rather than a 5 MB transmission. This 2,500× reduction in bandwidth enables continued operations when links are degraded to narrowband fallbacks, contested by jamming, or operating under strict emissions control (EMCON).

### Semantic Understanding for Tactical Decision-Making

Raw geometric mapping tells you where objects are; semantic mapping tells you what they mean. By fusing camera-based perception with LiDAR structure, the HD map encodes not just that an obstacle exists, but whether it is a civilian vehicle, military fighting position, traversable vegetation, or impassable building. This semantic layer enables autonomous systems to make tactically-relevant decisions without operator intervention. A UGV can distinguish between a parked car (bypass) and an abandoned technical (report and avoid). An aerial drone can prioritize imaging of semantically-tagged points of interest while ignoring irrelevant terrain. Commanders viewing the fused map see the battlefield through a lens of tactical meaning rather than raw sensor data.

### Resilience Through Distributed Redundancy

When mapping responsibility is distributed across multiple platforms, the loss of any single asset does not eliminate the accumulated knowledge. Each platform maintains a local copy of the collaborative map, and the Merkle tree structure enables rapid verification that maps remain synchronized even after periods of disconnection. A drone that loses communications for ten minutes can rejoin the network, compare fingerprints, and receive only the tiles that changed during its absence. If a ground vehicle is destroyed, its observations persist in the maps of every platform it synchronized with. This architectural resilience directly supports operations in high-attrition environments where platform loss must be anticipated rather than avoided.

### Cross-Domain Integration

The standardized map format and synchronization protocol are agnostic to platform type, enabling true cross-domain integration. The same HD map that guides autonomous ground vehicles can be consumed by dismounted infantry with augmented reality displays, air defense systems plotting engagement zones, or artillery units planning fire missions. Semantic tags bridge the gap between sensor modalities—a thermal-identified heat signature from an aerial platform can be correlated with the LiDAR structure observed by a ground vehicle, creating a fused understanding neither platform could achieve alone.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         COLLABORATIVE MAPPING NETWORK                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌──────────────┐        ┌──────────────┐        ┌──────────────┐          │
│   │  Aerial UAV  │◄──────►│  Ground UGV  │◄──────►│ Command Post │          │
│   │              │        │              │        │              │          │
│   │ • 3D LiDAR   │  Mesh  │ • 3D LiDAR   │  Mesh  │ • Map Server │          │
│   │ • RGB Camera │  Net   │ • RGB Camera │  Net   │ • Fusion     │          │
│   │ • Thermal    │        │ • Wheel Odom │        │ • Planning   │          │
│   │ • IMU/GPS    │        │ • IMU        │        │ • C2 Overlay │          │
│   └──────┬───────┘        └──────┬───────┘        └──────┬───────┘          │
│          │                       │                       │                   │
│          ▼                       ▼                       ▼                   │
│   ┌──────────────┐        ┌──────────────┐        ┌──────────────┐          │
│   │ Local HD Map │        │ Local HD Map │        │  Global Map  │          │
│   │ + Semantics  │        │ + Semantics  │        │   Database   │          │
│   │ + Fingerprint│        │ + Fingerprint│        │ + Fusion     │          │
│   └──────────────┘        └──────────────┘        └──────────────┘          │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Camera-LiDAR Fusion Pipeline

### Sensor Suite

| Platform | LiDAR | Camera | Thermal | Notes |
|----------|-------|--------|---------|-------|
| Aerial UAV | Ouster OS1-64 (120m, 1.3M pts/s) | 4K RGB global shutter | LWIR 640×512 | Synchronized, calibrated |
| Ground UGV | Velodyne VLP-32C (200m, 600K pts/s) | Stereo RGB + depth | LWIR 640×512 | Multi-baseline stereo |
| Aerial UAV (alt) | Livox Mid-360 (70m, 200K pts/s) | 1080p RGB | Optional | Low-cost variant |

### Fusion Architecture

The camera-LiDAR fusion pipeline projects semantic labels from camera perception onto the geometric structure captured by LiDAR, creating a unified representation where each voxel carries both spatial accuracy and semantic meaning.

**Fusion Process:**
1. **Geometric backbone**: LiDAR provides precise 3D structure via SLAM (KISS-ICP / Fast-GICP)
2. **Semantic inference**: Camera images processed through RT-DETR (objects) + segmentation model (terrain/structures)
3. **Projection**: Camera semantics projected onto LiDAR points using extrinsic calibration
4. **Temporal fusion**: Multi-frame observations aggregated with confidence weighting
5. **Voxel encoding**: Fused point cloud discretized into semantically-labeled voxel grid

### Semantic Class Taxonomy

| Category | Classes | Source Priority |
|----------|---------|-----------------|
| **Terrain** | Road, trail, grass, sand, water, mud | Camera (texture) > LiDAR (geometry) |
| **Structures** | Building, wall, fence, bunker, tower | LiDAR (geometry) > Camera (texture) |
| **Vehicles** | Civilian car, truck, military vehicle, technical | Camera (classification) + LiDAR (dimensions) |
| **Personnel** | Dismounted, prone, group | Camera (detection) + thermal (confirmation) |
| **Vegetation** | Tree, bush, crops, dense forest | Camera (texture) + LiDAR (penetration) |
| **Tactical** | Fighting position, IED indicator, checkpoint | Camera (classification) + context |

### Semantic Confidence Model

Each voxel stores semantic labels with associated confidence derived from:
- **Observation count**: More observations increase confidence
- **Sensor agreement**: Camera + LiDAR + thermal consensus
- **View quality**: Distance, angle, occlusion, lighting
- **Temporal consistency**: Stable classifications over time

---

## Local HD Map Structure

### Map Representation

The local HD map uses a hierarchical tile-based structure optimized for partial transmission:

| Layer | Content | Update Rate |
|-------|---------|-------------|
| **Geometry** | Occupancy voxels, ground mesh | 10 Hz (LiDAR rate) |
| **Semantics** | Class labels, confidence scores | 5 Hz (camera rate) |
| **Dynamic** | Tracked moving objects | 10 Hz |
| **Tactical** | POIs, annotations, overlays | Event-driven |

### Voxel Encoding

| Field | Bits | Description |
|-------|------|-------------|
| Occupancy | 1 | Occupied/free |
| Confidence | 4 | Observation count (log scale) |
| Semantic class | 6 | 64 classes (see taxonomy above) |
| Semantic confidence | 4 | Classification certainty |
| Height variance | 5 | Terrain roughness |
| Reflectivity | 4 | LiDAR intensity (normalized) |
| Timestamp | 8 | Last observation (relative, 256 increments) |
| **Total** | **32 bits** | 4 bytes per voxel |

### Tile Specifications

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Tile size | 50m × 50m | Balance between granularity and sync overhead |
| Voxel resolution | 0.2m | Sufficient for navigation, manageable size |
| Max height | 30m | Covers most structures |
| Typical tile size | 50-200 KB | After octree compression |

---

## Fingerprint Hashing System

### Purpose

Fingerprint hashes enable efficient synchronization by reducing map comparison to constant-time hash comparison. Only tiles with differing fingerprints require data transfer.

**Key Properties:**
- **256-bit SHA-256 hash** per tile for integrity and uniqueness
- **Merkle tree** structure enables partial verification of large maps
- **Geometric + semantic signature** captures both structure and meaning
- **Ed25519 signatures** authenticate message origin

### Fingerprint Composition

Each tile fingerprint incorporates:
1. Geometric eigenvalues (rotation-invariant shape descriptor)
2. Semantic class histogram (distribution of terrain/object types)
3. Occupancy density pyramid (multi-scale spatial structure)
4. Tile coordinates and version number

### Bandwidth Comparison

| Approach | 50-Tile Map Size | Suitable Bandwidth |
|----------|------------------|-------------------|
| Raw point cloud | ~500 MB | Fiber only |
| Compressed tiles | ~5 MB | High-bandwidth tactical |
| Fingerprints only | ~2 KB | Any tactical link |
| Delta sync (10% changed) | ~500 KB | Degraded tactical |

---

## Cross-Platform Synchronization Protocol

### Sync State Machine

```
IDLE ──► ANNOUNCE ──► COMPARE ──► TRANSFER ──► COMPLETE
  ▲                                                 │
  └─────────────────────────────────────────────────┘
```

1. **IDLE**: Await sync interval or trigger event
2. **ANNOUNCE**: Broadcast fingerprint message (Merkle root + tile hashes)
3. **COMPARE**: Identify tiles that differ between platforms
4. **TRANSFER**: Exchange compressed tile data for differing tiles
5. **COMPLETE**: Update local map, return to IDLE

### Protocol Messages

| Message | Direction | Size | Purpose |
|---------|-----------|------|---------|
| `MAP_ANNOUNCE` | Broadcast | ~2 KB | Share current map fingerprint |
| `MAP_QUERY` | Unicast | 100 B | Request specific tile fingerprints |
| `MAP_DIFF_REQ` | Unicast | 200 B | Request tiles that differ |
| `MAP_TILE_DATA` | Unicast | 50-200 KB | Compressed tile payload |
| `MAP_ACK` | Unicast | 50 B | Confirm tile receipt |

### Bandwidth Management

| Data Link | Bandwidth | Sync Interval | Tiles/Sync | Coverage Rate |
|-----------|-----------|---------------|------------|---------------|
| Tactical mesh (high) | 2 Mbps | 1 sec | 10 tiles | 2,500 m²/sec |
| Tactical mesh (degraded) | 256 kbps | 5 sec | 2 tiles | 100 m²/sec |
| Satellite (BLOS) | 64 kbps | 30 sec | 1 tile | 83 m²/min |
| Radio relay | 1 Mbps | 2 sec | 5 tiles | 625 m²/sec |

---

## Platform-Specific Considerations

### Aerial UAV

| Aspect | Consideration |
|--------|---------------|
| **Perspective** | Overhead view, good for building footprints, roads, area coverage |
| **Semantic strength** | Wide-area classification, vehicle/personnel detection at range |
| **LiDAR density** | Lower due to altitude, but covers large swath |
| **Priority tiles** | Ahead of ground vehicles, around objectives, along routes |

### Ground UGV

| Aspect | Consideration |
|--------|---------------|
| **Perspective** | Ground-level, good for facades, obstacles, traversability detail |
| **Semantic strength** | Close-range classification, texture detail, under-canopy visibility |
| **LiDAR density** | High at close range, detailed obstacle mapping |
| **Priority tiles** | Immediate path, flank security, choke points |

### Map Fusion Strategy

**Fusion Rules:**
1. Ground observations override aerial for voxels within 2m of ground plane (better angle, higher density)
2. Aerial observations fill gaps in ground coverage (wider field of view)
3. Semantic labels use highest-confidence source regardless of platform
4. Conflict resolution prefers most recent observation when confidence is equal
5. Thermal detections elevate confidence for personnel/vehicle classifications

---

## Latency Requirements

| Requirement | Max Latency | Rationale |
|-------------|-------------|-----------|
| LiDAR scan to local map update | 200ms | Real-time navigation |
| Camera semantic fusion | 150ms | Aligned with perception pipeline |
| Fingerprint computation | 50ms | Doesn't block navigation |
| Map announcement broadcast | 1 sec | Balance freshness vs bandwidth |
| Tile transfer (single) | 5 sec | Background, non-blocking |
| Full map sync (50 tiles) | 60 sec | Complete area coverage |

---

## Security Considerations

| Threat | Mitigation |
|--------|------------|
| Map spoofing | Ed25519 signatures on all messages |
| Replay attacks | Timestamps + sequence numbers |
| Eavesdropping | Optional encryption layer (ChaCha20) |
| Denial of service | Rate limiting, priority queues |
| Malicious tiles | Plausibility checks, outlier detection |

---

## Open Questions

- [ ] Optimal tile size for different operational tempos?
- [ ] How to handle GNSS-denied alignment between platforms?
- [ ] Priority scheme for contested bandwidth scenarios?
- [ ] Semantic class taxonomy alignment across joint/coalition forces?
- [ ] Loop closure across multi-platform observations?
- [ ] Thermal-visual-LiDAR registration in dynamic thermal environments?

---

## Implementation References

Detailed implementation specifications (code, message formats, node configurations) are maintained in the repository:
- `src/navigation/hd_map/` - Map building and tile management
- `src/navigation/fingerprint/` - Hash computation and Merkle tree
- `src/communication/map_sync/` - Synchronization protocol
- `msgs/ros2/HDMap*.msg` - Custom message definitions

---

## Related Documents

- [Latency Requirements](../latency_requirements.md) - Platform timing budgets
- [Perception Architecture](../perception_architecture.md) - Camera-LiDAR fusion pipeline
- [Swarm Operations](swarm_operations.md) - Multi-agent coordination
- [ISR Use Case](isr.md) - Intelligence gathering integration
