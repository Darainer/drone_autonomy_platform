"""
ROS2 domain agent Temporal worker.
Handles activities for all domain agents (perception, nav, control, etc.)
on the ros2-dev task queue.

Start with: python -m agents.ros2_worker.worker
"""

import asyncio
import os
from temporalio.client import Client
from temporalio.worker import Worker

from agents.orchestrator.activities import run_domain_agent


async def main():
    temporal_address = os.environ.get("TEMPORAL_ADDRESS", "localhost:7233")
    print(f"🏗️ Connecting to Temporal at {temporal_address}...")

    client = await Client.connect(temporal_address)

    worker = Worker(
        client,
        task_queue="ros2-dev",
        activities=[run_domain_agent],
    )

    print("🏗️ ROS2 worker started. Listening on queue: ros2-dev")
    await worker.run()


if __name__ == "__main__":
    asyncio.run(main())
