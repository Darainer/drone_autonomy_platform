FROM ros:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-humble-mavros-msgs \
    ros-humble-nav2-core \
    ros-humble-vision-msgs \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY . src/drone_autonomy_platform/

# drone_autonomy_platform (top-level) uses plain cmake add_subdirectory and is not a colcon package.
# common is a header-only library with no dependents yet.
# perception builds on x86 (node only); Isaac ROS + OAK-D driver are runtime-only on Jetson.
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
        --merge-install \
        --base-paths src/drone_autonomy_platform/msgs src/drone_autonomy_platform/src \
        --packages-ignore common \
        --cmake-args -DBUILD_TESTING=OFF

# Overwrite the entrypoint to source both ROS2 and the built workspace
RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/humble/setup.bash\nsource /ws/install/setup.bash\nexec "$@"\n' \
    > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh
