FROM ros:humble

# Gazebo + PX4 SITL deps
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-ros-gz \
    && pip3 install --no-cache-dir temporalio anthropic \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY agents/ /app/agents/

CMD ["python3", "-m", "agents.sim_worker.worker"]
