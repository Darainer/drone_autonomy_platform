FROM nvcr.io/nvidia/pytorch:24.01-py3

RUN pip install --no-cache-dir \
    temporalio \
    anthropic \
    ultralytics \
    onnx \
    onnxruntime-gpu

WORKDIR /app

COPY agents/ /app/agents/

CMD ["python", "-m", "agents.ml_worker.worker"]
