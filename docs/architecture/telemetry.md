# Telemetry & Video Downlink Architecture

This document defines the communication architecture between the drone and ground station, including command/control telemetry and video streaming.

---

## Overview

The platform uses **two independent data links**:

| Link | Purpose | Protocol | Hardware |
|------|---------|----------|----------|
| **Telemetry** | Commands, status, waypoints | MAVLink | SiK Radio 433MHz |
| **Video** | Camera feed, AI overlays | RTP/H.264 | WiFi / WFB-ng / LTE |

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           GROUND STATION (Laptop)                           │
│  ┌──────────────────┐    ┌──────────────────┐    ┌──────────────────────┐   │
│  │  QGroundControl  │    │  Video Viewer    │    │  SiK Radio (USB)     │   │
│  │  (Telemetry/Cmd) │    │  (GStreamer/Web) │    │  433MHz Receiver     │   │
│  └────────┬─────────┘    └────────┬─────────┘    └──────────┬───────────┘   │
│           │                       │                         │               │
└───────────┼───────────────────────┼─────────────────────────┼───────────────┘
            │ USB                   │ WiFi/5.8GHz             │ 433MHz RF
            │                       │                         │
┌───────────┼───────────────────────┼─────────────────────────┼───────────────┐
│ DRONE     │                       │                         │               │
│           │                       ▼                         ▼               │
│           │            ┌──────────────────┐      ┌──────────────────┐       │
│           │            │  Jetson Orin     │      │  Pixhawk 6X      │       │
│           │            │  ┌────────────┐  │      │      (PX4)       │       │
│           │            │  │  OAK-D     │  │      └────────┬─────────┘       │
│           │            │  │  Camera    │  │               │                 │
│           │            │  └────────────┘  │               │ UART            │
│           │            │  ┌────────────┐  │◄──────────────┘                 │
│           │            │  │ GStreamer  │  │  MAVROS                         │
│           │            │  │ H.264→UDP  │  │                                 │
│           │            │  └────────────┘  │                                 │
│           │            │  ┌────────────┐  │                                 │
│           │            │  │ WiFi/WFB   │──┼─────── Video stream             │
│           │            │  └────────────┘  │                                 │
│           │            └──────────────────┘                                 │
│           │                     │                                           │
│           │                     │ USB2SERIAL                                │
│           │                     ▼                                           │
│           │            ┌──────────────────┐                                 │
│           └───────────▶│  SiK Radio       │◄─── Telemetry ──────────────────│
│                        │  433MHz TX       │                                 │
│                        └──────────────────┘                                 │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## MAVLink Telemetry (Command & Control)

### Hardware

| Component | Specification |
|-----------|---------------|
| Radio | SiK Telemetry Radio V3 |
| Frequency | 433MHz (EU) or 915MHz (US) |
| Bandwidth | 57.6 kbps |
| Range | 1-2km (stock antennas), 5km+ (upgraded) |
| Protocol | MAVLink 2.0 |

### Physical Connections

```
Pixhawk 6X                    Ground Station
┌─────────────┐               ┌─────────────┐
│   TELEM1    │───RF Link────│   USB       │
│   (UART)    │   433MHz     │   (Serial)  │
└─────────────┘               └─────────────┘
      │
      │ TELEM2 (UART)
      ▼
┌─────────────┐
│ USB2SERIAL  │──USB──▶ Jetson Orin (MAVROS)
└─────────────┘
```

### Pixhawk Serial Port Usage

| Pixhawk Port | Device | Protocol | Role |
|--------------|--------|----------|------|
| `TELEM1` | SiK radio | MAVLink | Ground-station telemetry |
| `TELEM2` | Jetson companion link | MAVLink | PX4 ↔ Orin companion communication |
| `TELEM3` | ELRS receiver | CRSF | RC control + telemetry back to transmitter |

`ELRS` is connected on `TELEM3` rather than `RC IN` because `CRSF` is a bidirectional UART protocol. `RC IN` remains unused unless switching to a legacy receiver protocol such as `SBUS`.

For the PX4 parameter values corresponding to this serial layout, see
[`px4_setup.md`](./px4_setup.md).

### Data Carried

| Direction | Data |
|-----------|------|
| **Downlink** (Drone → GCS) | GPS position, attitude, altitude, battery, flight mode, heartbeat, sensor status |
| **Uplink** (GCS → Drone) | Waypoints, mode changes, arm/disarm, RTL, parameter updates |

### Configuration

For the PX4 parameters used by this project, see
[`px4_setup.md`](./px4_setup.md).

---

## Video Downlink

### Phased Approach

| Phase | Solution | Range | Latency | Cost |
|-------|----------|-------|---------|------|
| 1. Development | Standard WiFi | 100-500m | 100-300ms | $0 |
| 2. Field Testing | LTE + VPN | Cell coverage | 100-500ms | $20/mo |
| 3. Production | WFB-ng (5.8GHz) | 5-15km | 60-120ms | ~$100 |

---

## Phase 1: WiFi Streaming (Development)

### Overview

Use Jetson's built-in WiFi or an external USB adapter for short-range streaming during development and testing.

### Hardware Options

| Option | Specification | Notes |
|--------|---------------|-------|
| Built-in WiFi | 802.11ac | Sufficient for dev |
| Intel AX210 USB | WiFi 6E, 5GHz | Better range/throughput |

### GStreamer Pipeline

**On Jetson (transmit):**
```bash
# Hardware-accelerated H.264 encoding
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=1280,height=720,framerate=30/1 ! \
  nvvidconv ! nvv4l2h264enc bitrate=4000000 ! \
  h264parse ! rtph264pay ! \
  udpsink host=<LAPTOP_IP> port=5600
```

**On Laptop (receive):**
```bash
# Decode and display
gst-launch-1.0 udpsrc port=5600 ! \
  application/x-rtp,encoding-name=H264 ! \
  rtph264depay ! avdec_h264 ! autovideosink
```

### QGroundControl Integration

QGroundControl natively supports RTP video on port 5600:
1. Go to **Application Settings** → **Video**
2. Set **Video Source** to **UDP h.264 Video Stream**
3. Set **UDP Port** to **5600**

---

## Phase 2: LTE/Cellular (Extended Range)

### Overview

Use a USB LTE modem on the Jetson with a VPN for direct connectivity. Good for missions within cell coverage.

### Hardware

| Component | Specification | ~Price |
|-----------|---------------|--------|
| USB Modem | Quectel RM520N / Sierra EM7455 | $50-100 |
| SIM Card | Data plan (recommend unlimited) | $20/mo |

### Network Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│ DRONE                                                           │
│  Jetson Orin                                                    │
│  ├── USB LTE Modem ──▶ Cell Tower ──▶ Internet                  │
│  │                                           │                  │
│  └── Tailscale/ZeroTier VPN ◀────────────────┘                  │
│              │                                                  │
│              │ Private IP (e.g., 100.x.x.x)                     │
└──────────────┼──────────────────────────────────────────────────┘
               │
               │ Direct connection (no port forwarding)
               ▼
┌──────────────────────────────────────────────────────────────────┐
│ GROUND STATION                                                   │
│  Laptop                                                          │
│  └── Tailscale/ZeroTier VPN ──▶ Internet ◀── Cell/WiFi          │
│              │                                                   │
│              │ Private IP (e.g., 100.x.x.x)                      │
└──────────────────────────────────────────────────────────────────┘
```

### Setup

```bash
# Install Tailscale on Jetson
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up

# Install on laptop
# (same process)

# Stream to Tailscale IP
gst-launch-1.0 ... udpsink host=100.x.x.x port=5600
```

### Latency Considerations

| Network | Typical Latency |
|---------|-----------------|
| 4G LTE | 50-100ms |
| 5G | 10-30ms |
| Total (with encoding) | 100-200ms |

**Note:** Not suitable for FPV piloting. Use for monitoring/ISR only.

---

## Phase 3: WFB-ng (Production Open Source)

### Overview

**WFB-ng (WiFi Broadcast Next Generation)** is an open source video link that uses WiFi hardware in monitor mode to create a broadcast link similar to analog FPV.

**Key features:**
- Unidirectional broadcast (no TCP handshake)
- Forward Error Correction (FEC)
- AES encryption
- Works through obstacles better than standard WiFi
- Can carry video + MAVLink over the same link

### Hardware

| Component | Recommendation | ~Price |
|-----------|----------------|--------|
| Air WiFi Adapter | ALFA AWUS036ACH (RTL8812AU) | $30 |
| Ground WiFi Adapter | ALFA AWUS036ACH (RTL8812AU) | $30 |
| Air Antenna | 5dBi omnidirectional (RP-SMA) | $15 |
| Ground Antenna | 9-14dBi directional patch | $25 |
| **Total** | | **~$100** |

### Why RTL8812AU

| Feature | Requirement | RTL8812AU |
|---------|-------------|-----------|
| 5.8GHz support | ✓ | ✓ |
| Monitor mode | ✓ | ✓ |
| Packet injection | ✓ | ✓ |
| TX power | High | 500mW+ |
| Linux driver | Stable | ✓ (aircrack-ng) |

### Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ DRONE                                                                       │
│                                                                             │
│  OAK-D ──USB──▶ Jetson Orin                                                 │
│                    │                                                        │
│                    ├── ROS2/Perception processing                           │
│                    │                                                        │
│                    ├── GStreamer H.264 encode (nvv4l2h264enc)               │
│                    │         │                                              │
│                    │         ▼                                              │
│                    └── wfb_tx ──▶ RTL8812AU (5.8GHz) ───── RF ──────────┐   │
│                                        │                                │   │
│                                   [5dBi omni]                           │   │
└─────────────────────────────────────────────────────────────────────────│───┘
                                                                          │
                                                              5.8GHz broadcast
                                                                          │
┌─────────────────────────────────────────────────────────────────────────│───┐
│ GROUND STATION                                                          │   │
│                                           [14dBi patch]                 │   │
│                                                │                        │   │
│  RTL8812AU (5.8GHz) ◀── wfb_rx ◀───────────────┘                        │   │
│       │                                                                     │
│       ▼                                                                     │
│  GStreamer decode ──▶ QGroundControl / Custom UI                            │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Performance

| Metric | Value |
|--------|-------|
| Range | 2-7km (omnidirectional), 10-20km (directional) |
| Latency | 60-120ms (with FEC) |
| Bandwidth | Up to 12 Mbps usable |
| Resolution | 1080p/30fps or 720p/60fps typical |

### Installation

**On Jetson (air side):**

```bash
# Install dependencies
sudo apt update
sudo apt install -y libpcap-dev libsodium-dev python3-pip

# Clone and build WFB-ng
git clone https://github.com/svpcom/wfb-ng.git
cd wfb-ng
make
sudo make install

# Generate encryption keys (run once)
wfb_keygen
# Creates /etc/drone.key (air) and /etc/gs.key (ground)
# Copy gs.key to ground station

# Install RTL8812AU driver
git clone https://github.com/aircrack-ng/rtl8812au.git
cd rtl8812au
sudo make dkms_install
```

**Configure adapter (air side):**

```bash
# Find adapter interface name
ip link show  # e.g., wlan1

# Put in monitor mode
sudo ip link set wlan1 down
sudo iw dev wlan1 set monitor none
sudo ip link set wlan1 up
sudo iw dev wlan1 set channel 149 HT40+  # 5.8GHz channel
```

**On Laptop (ground side):**

```bash
# Same driver and WFB-ng installation
# Copy gs.key from Jetson to /etc/gs.key

# Configure adapter (same commands)
sudo ip link set wlan1 down
sudo iw dev wlan1 set monitor none
sudo ip link set wlan1 up
sudo iw dev wlan1 set channel 149 HT40+
```

### Running the Link

**Air side (Jetson):**

```bash
# Terminal 1: Start WFB-ng transmitter
wfb_tx -p 0 -u 5600 -K /etc/drone.key -B 20 -M 4 wlan1

# Terminal 2: Start video pipeline
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=1280,height=720,framerate=30/1 ! \
  nvvidconv ! nvv4l2h264enc bitrate=4000000 ! \
  h264parse ! rtph264pay ! \
  udpsink host=127.0.0.1 port=5600
```

**Ground side (Laptop):**

```bash
# Terminal 1: Start WFB-ng receiver
wfb_rx -p 0 -c 127.0.0.1 -u 5600 -K /etc/gs.key wlan1

# Terminal 2: Receive and display
gst-launch-1.0 udpsrc port=5600 ! \
  application/x-rtp,encoding-name=H264 ! \
  rtph264depay ! avdec_h264 ! autovideosink
```

### WFB-ng Parameters

| Parameter | Description | Recommended |
|-----------|-------------|-------------|
| `-p` | Stream ID (0-127) | 0 for video |
| `-u` | UDP port | 5600 |
| `-K` | Key file | /etc/drone.key or /etc/gs.key |
| `-B` | Bandwidth (MHz) | 20 (standard) or 40 (high throughput) |
| `-M` | MCS index (0-7) | 4 (balanced), 2 (robust), 6 (high speed) |

### Dual Stream (Video + MAVLink)

```bash
# Air side: Video on stream 0, MAVLink on stream 1
wfb_tx -p 0 -u 5600 -K /etc/drone.key wlan1 &  # Video
wfb_tx -p 1 -u 14550 -K /etc/drone.key wlan1 & # MAVLink

# Ground side
wfb_rx -p 0 -c 127.0.0.1 -u 5600 -K /etc/gs.key wlan1 &  # Video
wfb_rx -p 1 -c 127.0.0.1 -u 14550 -K /etc/gs.key wlan1 & # MAVLink
```

---

## Comparison: Video Link Options

| Feature | WiFi | LTE | WFB-ng | DJI O3 |
|---------|------|-----|--------|--------|
| **Range** | 100-500m | Cell coverage | 5-15km | 10km+ |
| **Latency** | 100-300ms | 100-500ms | 60-120ms | 28-40ms |
| **Cost** | $0 | $20/mo | ~$100 | $580+ |
| **Open Source** | ✓ | ✓ | ✓ | ✗ |
| **Runs on Jetson** | ✓ | ✓ | ✓ | Separate |
| **Customizable** | ✓ | ✓ | ✓ | ✗ |
| **Dual-stream** | ✓ | ✓ | ✓ | ✗ |
| **Obstacle penetration** | Poor | Good | Medium | Good |

---

## ROS2 Integration

### Video Bridge Node

```
┌────────────────────────────────────────────────────────────────┐
│ Jetson Orin - ROS2 Pipeline                                   │
│                                                                │
│  /camera/image_raw ──▶ /perception_node ──▶ /detections        │
│         │                     │                                │
│         │                     ▼                                │
│         │              overlay_node (draw bboxes)              │
│         │                     │                                │
│         └─────────────────────┴──▶ gst_bridge_node             │
│                                          │                     │
│                                          ▼                     │
│                                   GStreamer pipeline           │
│                                          │                     │
│                                          ▼                     │
│                                   wfb_tx / udpsink             │
└────────────────────────────────────────────────────────────────┘
```

### Example: gst_bridge Launch

```python
# launch/video_stream.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gst_bridge',
            executable='ros_to_gst',
            parameters=[{
                'input_topic': '/camera/image_raw/compressed',
                'gst_pipeline': 'appsrc ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5600',
            }],
        ),
    ])
```

---

## Regulatory Notes

| Region | 5.8GHz Restrictions |
|--------|---------------------|
| USA (FCC) | 1W EIRP max, no license required |
| EU (ETSI) | 25mW indoor, 1W outdoor with DFS |
| Other | Check local regulations |

**Note:** WFB-ng in monitor mode may not comply with all regulations. For commercial use, verify local requirements.

---

## References

- [WFB-ng GitHub](https://github.com/svpcom/wfb-ng)
- [RTL8812AU Driver](https://github.com/aircrack-ng/rtl8812au)
- [GStreamer NVIDIA Plugins](https://docs.nvidia.com/jetson/l4t-multimedia/group__gst__nvvideo__encoder.html)
- [QGroundControl Video Setup](https://docs.qgroundcontrol.com/master/en/SettingsView/VideoSettings.html)
- [MAVLink Protocol](https://mavlink.io/en/)
