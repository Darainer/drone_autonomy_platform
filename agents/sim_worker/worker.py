"""
Simulation Temporal worker.
Runs Gazebo + PX4 SITL test activities.

Start with: python -m agents.sim_worker.worker
"""

import asyncio
import os
from temporalio.client import Client
from temporalio.worker import Worker

from agents.orchestrator.activities import run_simulation


async def main():
    temporal_address = os.environ.get("TEMPORAL_ADDRESS", "localhost:7233")
    print(f"🧪 Connecting to Temporal at {temporal_address}...")

    client = await Client.connect(temporal_address)

    worker = Worker(
        client,
        task_queue="simulation",
        activities=[run_simulation],
    )

    print("🧪 Simulation worker started. Listening on queue: simulation")
    await worker.run()


if __name__ == "__main__":
    asyncio.run(main())
