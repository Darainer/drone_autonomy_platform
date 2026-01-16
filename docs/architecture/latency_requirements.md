# Latency Requirements for ISR

This document defines the latency requirements for the Intelligence, Surveillance, and Reconnaissance (ISR) use case. Latency is a critical factor in the performance of an ISR system, as it determines how quickly the drone can react to new information and threats.

## 1. End-to-End Latency Requirements

End-to-end latency is defined as the time from when a photon hits the sensor to when the drone takes an action based on that information.

| ID | Requirement | Max Latency (ms) | Rationale |
|---|---|---|---|
| E2E-1 | **Sensor to GCS Display** | 500 | The time from when an image is captured to when it is displayed on the ground control station (GCS). This is important for real-time situational awareness. |
| E2E-2 | **Sensor to Action (Threat Response)** | 250 | The time from when a threat is detected to when the drone takes a defensive action (e.g., maneuvers to avoid a missile). This is a critical safety requirement. |
| E2E-3 | **Sensor to Action (Target Tracking)** | 300 | The time from when a target is detected to when the drone adjusts its course to track the target. This is important for maintaining a lock on moving targets. |

## 2. Component-Level Latency Requirements

The end-to-end latency requirements can be broken down into the following component-level requirements:

| ID | Component | Max Latency (ms) | Rationale |
|---|---|---|---|
| COMP-1 | **Camera** | 30 | The time from when the image is captured to when it is available on the MIPI-CSI bus. |
| COMP-2 | **Image Processing** | 50 | The time to perform any necessary image processing, such as debayering and color correction. |
| COMP-3 | **Perception (Object Detection)** | 100 | The time to run the object detection model on the image. |
| COMP-4 | **Perception (Object Tracking)** | 50 | The time to track the detected objects from one frame to the next. |
| COMP-5 | **Decision Making** | 20 | The time for the mission manager to make a decision based on the perceived information. |
| COMP-6 | **Control Command Generation** | 10 | The time to generate a new control command (e.g., attitude, velocity). |
| COMP-7 | **Flight Controller** | 10 | The time for the flight controller to process the control command and send it to the motors. |
| COMP-8 | **Data Link (Uplink)** | 40 | The time to transmit a command from the GCS to the drone. |
| COMP-9 | **Data Link (Downlink)** | 100 | The time to transmit a compressed video stream from the drone to the GCS. |

## 3. Platform Processing Requirements

The platform processing requirements define the latency requirements for the processing that happens on the drone's mission computer (e.g., NVIDIA Jetson Orin).

| ID | Requirement | Max Latency (ms) | Rationale |
|---|---|---|---|
| PLAT-1 | **Sensor Input to Perception Output** | 150 | The time from when the sensor data is available to when the perception module has finished processing it. This is the sum of COMP-2, COMP-3, and COMP-4. |
| PLAT-2 | **Perception Output to Control Command** | 30 | The time from when the perception module has finished processing to when a new control command is generated. This is the sum of COMP-5 and COMP-6. |
| PLAT-3 | **Total Onboard Processing** | 180 | The total time to process the sensor data and generate a new control command. This is the sum of PLAT-1 and PLAT-2. |

## 4. Latency Budget

The following table shows a sample latency budget for the "Sensor to Action (Threat Response)" requirement (E2E-2).

| Component | Max Latency (ms) |
|---|---|
| Camera | 30 |
| Image Processing | 50 |
| Perception (Object Detection) | 100 |
| Decision Making | 20 |
| Control Command Generation | 10 |
| Flight Controller | 10 |
| **Total** | **220** |

This budget shows that it is possible to meet the 250ms end-to-end latency requirement if each component meets its individual latency requirement.
