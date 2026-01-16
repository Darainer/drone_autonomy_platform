# Military Drone Use Cases and Perception Solutions

This document outlines common military use cases for autonomous drones and the camera-based perception functionalities required to solve them.

## Drone Autonomy Architecture

The drone autonomy architecture is a modular system designed for flexibility and extensibility. It consists of the following core components:

- **Perception:** This is the "eyes" of the drone. It processes data from various sensors, with a strong emphasis on cameras, to understand the environment.
- **Navigation:** This is the "brain" of the drone's movement. It handles localization (knowing where it is), mapping (building a map of the environment), and path planning (figuring out how to get from A to B).
- **Control:** This is the "muscles" of the drone. It takes the desired trajectory from the navigation module and translates it into motor commands.
- **Autonomy:** This is the "mission commander" of the drone. It contains the high-level logic that dictates the drone's overall behavior, such as following a mission plan or reacting to events.
- **Communication:** This is the "nervous system" of the drone. It handles communication with the ground control station and other drones.
- **Safety:** This is the "guardian angel" of the drone. It monitors the drone's health and the environment to prevent accidents.

## Military Use Cases and Perception Solutions

| Use Case | Description | Required Perception Functionality | Open-Source References |
|---|---|---|---|
| **Intelligence, Surveillance, and Reconnaissance (ISR)** | Gathering information about enemy forces and terrain. This is the most common use case for military drones. | - **Object Detection and Tracking:** To identify and track enemy vehicles, soldiers, and equipment. <br> - **Semantic Segmentation:** To classify different types of terrain (e.g., roads, buildings, forests). <br> - **Change Detection:** To identify changes in the environment over time (e.g., new construction, troop movements). | - [YOLOv8](https://github.com/ultralytics/ultralytics) <br> - [DeepSORT](https://github.com/nwojke/deep_sort) <br> - [MMSegmentation](https://github.com/open-mmlab/mmsegmentation) |
| **Target Acquisition and Designation** | Identifying and marking targets for attack by other assets (e.g., aircraft, artillery). | - **High-Resolution Object Detection:** To positively identify targets from a long distance. <br> - **Object Tracking:** To maintain a lock on the target as it moves. <br> - **3D Object Pose Estimation:** To determine the precise location and orientation of the target. | - [Swin Transformer](https://github.com/microsoft/Swin-Transformer) <br> - [PV-RCNN](https://github.com/open-mmlab/OpenPCDet) |
| **Battle Damage Assessment (BDA)** | Assessing the effectiveness of an attack. | - **Change Detection:** To compare the scene before and after the attack. <br> - **3D Reconstruction:** To create a 3D model of the target area to assess the extent of the damage. | - [OpenCV](https://github.com/opencv/opencv) <br> - [COLMAP](https://github.com/colmap/colmap) |
| **Convoy Escort and Protection** | Protecting a convoy of vehicles from attack. | - **Object Detection and Tracking:** To identify and track potential threats to the convoy. <br> - **Lane Detection:** To ensure the drone stays in its lane and doesn't collide with other vehicles. <br> - **Obstacle Avoidance:** To avoid colliding with obstacles on the road. | - [YOLOv8](https://github.com/ultralytics/ultralytics) <br> - [LaneNet](https://github.com/MaybeShewill-CV/lanenet-lane-detection) <br> - [Dynamic Window Approach](https://github.com/mherrerias/dwa) |
| **Search and Rescue (SAR)** | Finding and rescuing soldiers in distress. | - **Human Detection:** To find soldiers on the ground. <br> - **Thermal Imaging:** To find soldiers at night or in dense foliage. <br> - **Object Tracking:** To track the location of the soldier once they are found. | - [YOLOv8](https://github.com/ultralytics/ultralytics) <br> - [FLIR-Public-Models](https://github.com/FLIR/flir-public-models) |
| **Swarm Operations** | A large number of drones working together to achieve a common goal. | - **Relative Positioning:** To know the position of other drones in the swarm. <br> - **Collision Avoidance:** To avoid colliding with other drones in the swarm. <br> - **Visual Servoing:** To maintain a formation with other drones in the swarm. | - [Crazyflie Swarm](https://github.com/bitcraze/crazyflie-swarm-examples) <br> - [Crazyswarm](https://github.com/USC-ACTLab/crazyswarm) |
