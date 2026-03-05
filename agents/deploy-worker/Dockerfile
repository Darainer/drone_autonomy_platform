FROM python:3.12-slim

RUN apt-get update && apt-get install -y \
    docker.io \
    openssh-client \
    rsync \
    && pip install --no-cache-dir temporalio anthropic \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY agents/ /app/agents/

CMD ["python", "-m", "agents.deploy_worker.worker"]
