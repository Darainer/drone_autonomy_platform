FROM ros:humble

# Install Python deps
RUN apt-get update && apt-get install -y python3-pip && \
    pip3 install --no-cache-dir temporalio anthropic && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY agents/ /app/agents/

CMD ["python3", "-m", "agents.ros2_worker.worker"]
