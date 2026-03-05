"""
Deployment Temporal worker.
Cross-compile + push to Orin Nano.

Start with: python -m agents.deploy_worker.worker
"""

import asyncio
import os
from temporalio.client import Client
from temporalio.worker import Worker

from agents.orchestrator.activities import run_deploy


async def main():
    temporal_address = os.environ.get("TEMPORAL_ADDRESS", "localhost:7233")
    print(f"🚀 Connecting to Temporal at {temporal_address}...")

    client = await Client.connect(temporal_address)

    worker = Worker(
        client,
        task_queue="deployment",
        activities=[run_deploy],
    )

    print("🚀 Deploy worker started. Listening on queue: deployment")
    await worker.run()


if __name__ == "__main__":
    asyncio.run(main())
