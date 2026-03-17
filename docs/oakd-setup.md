# OAK-D Camera Setup (Jetson Orin)

## Hardware

- **Camera:** Luxonis OAK-D (Movidius MyriadX)
- **USB:** Connect via USB-C to the Orin (USB 3.0 port recommended)
- **Power:** The OAK-D barrel jack (5.5mm/2.1mm, center-positive, 5V 2A) is required on the Orin Nano — USB alone may not provide enough current to boot the Myriad X chip

## Host Setup (one-time)

The OAK-D USB device needs write permissions. This must be configured on the **Orin host**, not inside Docker.

### 1. Install udev rules

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 2. Verify the camera is detected

Unplug and replug the OAK-D, then:

```bash
cat /sys/bus/usb/devices/*/product 2>/dev/null | grep Movidius
```

Should output `Movidius MyriadX`.

### 3. Verify permissions

```bash
# Find the device node
BUS=$(grep -rl Movidius /sys/bus/usb/devices/*/product | head -1 | xargs dirname)
BUSNUM=$(cat $BUS/busnum)
DEVNUM=$(cat $BUS/devnum)
ls -la /dev/bus/usb/$(printf "%03d" $BUSNUM)/$(printf "%03d" $DEVNUM)
```

Should show `crw-rw-rw-` (mode 0666).

## Docker

The `docker-compose.yml` orin service already mounts USB devices:

```yaml
privileged: true
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0
volumes:
  - /dev/bus/usb:/dev/bus/usb
```

No additional Docker configuration is needed once the host udev rules are in place.

## Testing inside the container

```bash
# Start the camera node
ros2 run depthai_ros_driver camera_node

# In another terminal, check topics
ros2 topic list | grep oak
ros2 topic hz /oak/rgb/image_raw
```

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `X_LINK_UNBOOTED device` | Camera not powered or USB power insufficient | Connect 5V barrel jack |
| `Insufficient permissions` | Missing udev rules on host | Run udev setup above |
| No Movidius in sysfs | Camera not connected or USB cable faulty | Try different USB port/cable |
| `Failed to boot device` | Camera detected but can't initialize | Unplug, wait 5s, replug |
