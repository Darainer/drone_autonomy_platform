# PX4 Setup

This document tracks the PX4 parameters and port assignments required for the
drone autonomy platform.

It is focused on the current hardware layout:

- `TELEM1` -> SiK telemetry radio
- `TELEM2` -> Jetson Orin companion link
- `TELEM3` -> ELRS receiver via `CRSF`

---

## Port Allocation

| Pixhawk Port | Device | Protocol | Purpose |
|--------------|--------|----------|---------|
| `TELEM1` | SiK radio | MAVLink | Ground-station telemetry |
| `TELEM2` | Jetson Orin | MAVLink | Companion-computer link |
| `TELEM3` | ELRS receiver | CRSF | RC control + telemetry |
| `RC IN` | Unused | N/A | Reserved for legacy SBUS-style receivers |

Notes:

- `TELEM2` is the preferred PX4 companion-computer UART.
- `TELEM3` is used for ELRS because `CRSF` is a bidirectional UART protocol.
- `RC IN` is not used in the current design.

---

## Required PX4 Parameters

### TELEM1 - SiK Ground Telemetry

| Parameter | Value | Notes |
|-----------|-------|-------|
| `MAV_0_CONFIG` | `TELEM 1` | Assign MAVLink instance 0 to TELEM1 |
| `MAV_0_MODE` | `Normal` | Standard ground-station telemetry |
| `SER_TEL1_BAUD` | `57600` | Typical SiK baud rate |

### TELEM2 - Jetson Orin Companion Link

| Parameter | Value | Notes |
|-----------|-------|-------|
| `MAV_1_CONFIG` | `TELEM 2` | Assign MAVLink instance 1 to TELEM2 |
| `MAV_1_MODE` | `Onboard` | Companion-computer stream profile |
| `SER_TEL2_BAUD` | `921600` | High-rate serial link for MAVROS |
| `MAV_1_FORWARD` | `0` / `Disabled` | Do not forward MAVLink unnecessarily |
| `MAV_1_RATE` | `0` | Use PX4 default / half-max onboard stream rate |

Expected Orin-side device:

- `/dev/ttyUSB0` when using the UAV-DEV USB-to-serial adapter

Typical MAVROS URL:

```bash
fcu_url:=/dev/ttyUSB0:921600
```

### TELEM3 - ELRS Receiver

| Parameter | Value | Notes |
|-----------|-------|-------|
| `RC_CRSF_PRT_CFG` or equivalent | `TELEM 3` | Parameter name can vary by PX4 version |
| CRSF enablement | Enabled | Required for ELRS over CRSF |

Notes:

- ELRS should be configured as `CRSF`, not `SBUS`, for this project.
- Some PX4 builds/versions may require explicit CRSF module support.
- The exact parameter name can vary slightly across PX4 releases, so verify in
  QGroundControl on the target firmware version.

---

## Companion Computer Validation

After applying the parameters above and rebooting PX4:

1. Confirm the Orin sees the USB serial adapter:

```bash
ls -l /dev/ttyUSB0
```

2. Confirm MAVROS can open the link:

```bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600
```

3. Confirm ROS 2 receives FCU state:

```bash
ros2 topic echo /uas1/state
```

Expected result:

- MAVROS opens `/dev/ttyUSB0` successfully
- `/uas1/state` publishes
- PX4 heartbeat is visible

---

## QGroundControl Checklist

Use QGroundControl to verify:

1. `TELEM1` is assigned to SiK telemetry and matches the radio baud rate.
2. `TELEM2` is configured for MAVLink `Onboard` at `921600`.
3. `TELEM3` is configured for `CRSF` / ELRS.
4. Reboot PX4 after parameter changes.
5. Re-test the Orin companion link after reboot.

---

## Open Items

- Confirm the exact ELRS/CRSF parameter names for the PX4 firmware version used
  on the aircraft.
- Add any project-specific failsafe, RC, and flight-mode parameters once the
  airframe configuration is finalized.
