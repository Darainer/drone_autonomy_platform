# docker/sim/Dockerfile
#
# SITL simulation container for the drone autonomy platform.
# Stack: PX4 v1.14.3 SITL + Isaac Sim 4.2 + Pegasus Simulator + mavrospy
#
# Isaac Sim is NOT included in this image — it must be installed on the host
# and bind-mounted at /isaac-sim (see docker-compose.yml). This is because
# Isaac Sim requires GPU passthrough and is ~50 GB; baking it into the image
# is impractical.  The container provides everything else the simulation needs:
# PX4 SITL, ROS2 Humble, MAVROS, mavrospy, and the platform nodes.
#
# Prerequisites (host machine):
#   - Ubuntu 22.04
#   - NVIDIA driver 550+, RTX 3070 or better, 8 GB VRAM
#   - 32 GB RAM, 50 GB SSD
#   - NVIDIA Container Toolkit configured
#   - Isaac Sim 4.2 installed at ~/isaacsim (or override ISAACSIM_PATH)
#   - Pegasus Simulator cloned at ~/PegasusSimulator
#
# Build:
#   docker compose -f docker/sim/docker-compose.yml build
#
# Run:
#   docker compose -f docker/sim/docker-compose.yml up

FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# ── System dependencies ───────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS2 / MAVROS
    ros-humble-mavros \
    ros-humble-mavros-msgs \
    ros-humble-mavros-extras \
    # PX4 build tools
    make \
    cmake \
    git \
    python3-pip \
    python3-venv \
    ninja-build \
    # GStreamer (QGroundControl telemetry video)
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    # X11 forwarding (Isaac Sim GUI)
    libgl1-mesa-glx \
    libglib2.0-0 \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# ── Install geographic_msgs dataset (required by MAVROS) ─────────────────────
RUN ros2 run mavros install_geographiclib_datasets.py || true

# ── Python deps for PX4 SITL and Pegasus Simulator ───────────────────────────
RUN pip3 install --no-cache-dir \
    kconfiglib \
    jinja2 \
    jsonschema \
    pyros-genmsg \
    packaging \
    toml \
    numpy \
    future \
    "empy==3.3.4" \
    pymavlink \
    # Pegasus Simulator Python API
    "isaacsim-pegasus-simulator" \
    # MAVLink scripting
    MAVProxy

# ── Clone PX4 v1.14.3 and compile SITL ───────────────────────────────────────
# We build only the none (headless) target here; Pegasus Simulator provides
# the physics / rendering backend, so Gazebo is not needed in this container.
WORKDIR /opt
RUN git clone --depth 1 --branch v1.14.3 --recurse-submodules \
        https://github.com/PX4/PX4-Autopilot.git px4 && \
    /opt/px4/Tools/setup/ubuntu.sh --no-nuttx && \
    cd /opt/px4 && make px4_sitl_default none

ENV PX4_SRC=/opt/px4

# ── Clone mavrospy (already present in Dockerfile.dev; keep in sync) ─────────
RUN git clone --depth 1 https://github.com/vss2sn/mavrospy.git /opt/mavrospy

# ── ROS2 workspace (platform nodes) ──────────────────────────────────────────
# The platform source is bind-mounted at /ws/src at runtime.
ENV ROS_WS=/ws
WORKDIR /ws
COPY ../../src /ws/src
# msgs package must be built first
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select drone_autonomy_msgs && \
    colcon build --symlink-install

# ── Isaac Sim integration ─────────────────────────────────────────────────────
# Isaac Sim is bind-mounted from the host; expose its path as an env var so
# the launch scripts and scenario scripts can find it.
ENV ISAACSIM_PATH=/isaac-sim
ENV ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"

# Pegasus Simulator Python package — install into Isaac Sim's bundled Python
# (done at runtime via entrypoint because the bind mount isn't available at
# build time)
COPY docker/sim/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
