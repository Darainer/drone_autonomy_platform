"""
Orchestrator Temporal worker.
Runs workflows and orchestrator-level activities.

Start with: python -m agents.orchestrator.worker
"""

import asyncio
import os
from temporalio.client import Client
from temporalio.worker import Worker

from agents.orchestrator.workflows import (
    NewFeatureWorkflow,
    DeployWorkflow,
    ModelUpdateWorkflow,
)
from agents.orchestrator.activities import (
    analyze_intent,
    run_domain_agent,
    run_code_review,
    run_simulation,
    run_deploy,
    update_docs,
)


async def main():
    temporal_address = os.environ.get("TEMPORAL_ADDRESS", "localhost:7233")
    print(f"🎯 Connecting to Temporal at {temporal_address}...")

    client = await Client.connect(temporal_address)

    # Register workflows and activities on the orchestrator queue
    worker = Worker(
        client,
        task_queue="orchestrator",
        workflows=[
            NewFeatureWorkflow,
            DeployWorkflow,
            ModelUpdateWorkflow,
        ],
        activities=[
            analyze_intent,
            run_code_review,
            update_docs,
        ],
    )

    print("🎯 Orchestrator worker started. Listening on queue: orchestrator")
    await worker.run()


if __name__ == "__main__":
    asyncio.run(main())
