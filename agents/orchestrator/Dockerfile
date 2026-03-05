FROM python:3.12-slim

WORKDIR /app

# Install Temporal SDK and Anthropic client
RUN pip install --no-cache-dir \
    temporalio \
    anthropic

# Copy agent code
COPY agents/ /app/agents/

CMD ["python", "-m", "agents.orchestrator.worker"]
