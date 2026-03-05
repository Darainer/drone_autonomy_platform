"""
ML Pipeline Temporal worker.
Handles model training, export, and optimization activities.

Start with: python -m agents.ml_worker.worker
"""

import asyncio
import os
from temporalio.client import Client
from temporalio.worker import Worker

from agents.orchestrator.activities import run_domain_agent


async def main():
    temporal_address = os.environ.get("TEMPORAL_ADDRESS", "localhost:7233")
    print(f"🔬 Connecting to Temporal at {temporal_address}...")

    client = await Client.connect(temporal_address)

    worker = Worker(
        client,
        task_queue="ml-pipeline",
        activities=[run_domain_agent],
    )

    print("🔬 ML Pipeline worker started. Listening on queue: ml-pipeline")
    await worker.run()


if __name__ == "__main__":
    asyncio.run(main())
