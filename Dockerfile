FROM ros:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-humble-mavros-msgs \
    ros-humble-nav2-core \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY . src/drone_autonomy_platform/

# perception requires Isaac ROS (NVIDIA Jetson apt registry) and is excluded here.
# drone_autonomy_platform (top-level) uses plain cmake add_subdirectory and is not a colcon package.
# common is a header-only library with no dependents yet.
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
        --merge-install \
        --packages-ignore drone_autonomy_platform common perception \
        --cmake-args -DBUILD_TESTING=OFF

# Overwrite the entrypoint to source both ROS2 and the built workspace
RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/humble/setup.bash\nsource /ws/install/setup.bash\nexec "$@"\n' \
    > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh
