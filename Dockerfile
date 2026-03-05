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
# common is a header-only library with no dependents; skipped for now.
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
        --packages-ignore drone_autonomy_platform common perception \
        --cmake-args -DBUILD_TESTING=OFF

# Source the built workspace in every container invocation
RUN sed -i 's|exec "\$@"|source /ws/install/setup.bash; exec "$@"|' /ros_entrypoint.sh
