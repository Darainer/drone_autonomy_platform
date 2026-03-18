# Drone Hardware

## Frame

| Component | Specification |
|-----------|---------------|
| Frame | Tarot 650 Sport (Carbon Fiber) |
| Configuration | Quadcopter, X-frame |
| Prop-to-prop | 650 mm |

## Flight Controller

| Component | Specification |
|-----------|---------------|
| Autopilot | Holybro Pixhawk 6X (v2A) |
| Firmware | PX4 |
| RC Link | RadioMaster Boxer + R88 (ELRS 2.4 GHz, CRSF on TELEM3) |
| Telemetry | SiK Telemetry Radio V3 (433 MHz) |
| Companion Link | Telem 2 → UAV-DEV USB2SERIAL → Jetson USB 3.1 |

## Companion Computer

| Component | Specification |
|-----------|---------------|
| Compute | NVIDIA Jetson Orin Nano (JetPack 6.x) |
| Primary Storage | 1TB Lexar NM790 NVMe SSD |
| Boot Storage | 128GB SanDisk Extreme A2 microSD |
| Camera | Luxonis OAK-D (RGB + Stereo Depth + IMU) via USB 3.1 |

## Power System (6S)

| Component | Specification |
|-----------|---------------|
| Battery | SLS XTRON 10,000 mAh 6S1P (22.2 V) |
| Power Splitter | XT90-S Anti-Spark Parallel Y-Cable |
| Regulation | Matek UBEC Duo (12 V / 4 A → Jetson, 5 V / 4 A → Peripherals) |
| Charging | ISDT K4 Smart Duo + XT60-to-XT90 Adapters |

## Wiring Overview

```
Battery (6S 22.2V)
    │
    XT90-S Y-Cable ──▶ ESCs ──▶ Motors (x4)
    │
    └── Matek UBEC Duo
         ├── 12V ──▶ Jetson Orin Nano
         └──  5V ──▶ Pixhawk 6X, OAK-D, peripherals

Pixhawk 6X
    ├── Telem 1 ──▶ SiK 433 MHz radio ──▶ GCS (MAVLink)
    ├── Telem 2 ──▶ USB2SERIAL ──▶ Jetson (MAVROS)
    ├── Telem 3 ◀── R88 ELRS Receiver (CRSF, 2.4 GHz)
    ├── GPS     ◀── GPS module
    └── PWM Out ──▶ ESCs

Jetson Orin Nano
    ├── USB 3.1 ◀── OAK-D (RGB + stereo + IMU)
    ├── USB 3.1 ◀── USB2SERIAL ◀── Pixhawk Telem 2
    ├── WiFi    ──▶ Video downlink / WFB-ng
    └── NVMe    ──▶ 1TB storage (logs, models, maps)
```

## Antenna Placement

| Antenna | Frequency | Location |
|---------|-----------|----------|
| SiK telemetry | 433 MHz | Top plate, vertical whip |
| R88 ELRS receiver | 2.4 GHz | Rear arm, diversity antennas |
| WiFi (video/data) | 2.4 / 5 GHz | Jetson onboard or external USB adapter |
| GPS | 1.575 GHz (L1) | Top plate, clear sky view |

## Serial Port Allocation

| Pixhawk Port | Connected Device | Purpose |
|--------------|------------------|---------|
| `TELEM1` | SiK 433 MHz radio | Ground-station MAVLink telemetry |
| `TELEM2` | USB2SERIAL → Jetson Orin | Companion-computer MAVLink link |
| `TELEM3` | R88 ELRS receiver | RC + telemetry via CRSF |
| `RC IN` | Unused | Reserved unless switching to SBUS-style receivers |

Notes:

- `TELEM3` is preferred for ELRS when using `CRSF`, because CRSF is a bidirectional UART protocol.
- `RC IN` is only needed for legacy one-way RC protocols such as SBUS/PPM.
- All Pixhawk `TELEM` ports are 3.3 V UART. Match voltage levels when wiring ELRS and companion serial adapters.

For PX4 parameter values associated with this wiring, see
[`px4_setup.md`](./px4_setup.md).
