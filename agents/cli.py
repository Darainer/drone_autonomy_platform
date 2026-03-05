#!/usr/bin/env python3
"""
CLI to trigger agent workflows.

Usage:
  # New feature
  python -m agents.cli feature "Add obstacle detection to perception pipeline"

  # Deploy
  python -m agents.cli deploy v0.3.0

  # Model update
  python -m agents.cli model "Retrain YOLOv8 with new crop dataset"

  # Approve a waiting workflow
  python -m agents.cli approve <workflow-id>

  # Check workflow status
  python -m agents.cli status <workflow-id>
"""

import asyncio
import sys
import os
from temporalio.client import Client

from agents.orchestrator.workflows import (
    NewFeatureWorkflow,
    DeployWorkflow,
    ModelUpdateWorkflow,
    FeatureRequest,
    DeployRequest,
)


async def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    command = sys.argv[1]
    temporal_address = os.environ.get("TEMPORAL_ADDRESS", "localhost:7233")
    client = await Client.connect(temporal_address)

    match command:
        case "feature":
            desc = " ".join(sys.argv[2:])
            if not desc:
                print("Usage: python -m agents.cli feature 'description'")
                sys.exit(1)

            # Generate a workflow ID from the description
            wf_id = f"feature-{desc[:40].replace(' ', '-').lower()}"

            handle = await client.start_workflow(
                NewFeatureWorkflow.run,
                FeatureRequest(description=desc),
                id=wf_id,
                task_queue="orchestrator",
            )
            print(f"✅ Started workflow: {wf_id}")
            print(f"   Monitor at: http://localhost:8080/namespaces/default/workflows/{wf_id}")

        case "deploy":
            version = sys.argv[2] if len(sys.argv) > 2 else "latest"
            wf_id = f"deploy-{version}"

            handle = await client.start_workflow(
                DeployWorkflow.run,
                DeployRequest(version=version),
                id=wf_id,
                task_queue="orchestrator",
            )
            print(f"✅ Started deploy workflow: {wf_id}")
            print(f"   Monitor at: http://localhost:8080/namespaces/default/workflows/{wf_id}")

        case "model":
            desc = " ".join(sys.argv[2:])
            wf_id = f"model-{desc[:30].replace(' ', '-').lower()}"

            handle = await client.start_workflow(
                ModelUpdateWorkflow.run,
                {"description": desc, "action": desc},
                id=wf_id,
                task_queue="orchestrator",
            )
            print(f"✅ Started ML workflow: {wf_id}")
            print(f"   Monitor at: http://localhost:8080/namespaces/default/workflows/{wf_id}")

        case "approve":
            wf_id = sys.argv[2] if len(sys.argv) > 2 else None
            if not wf_id:
                print("Usage: python -m agents.cli approve <workflow-id>")
                sys.exit(1)

            handle = client.get_workflow_handle(wf_id)

            # Try both signal types
            try:
                await handle.signal(NewFeatureWorkflow.approve)
                print(f"✅ Sent approval signal to {wf_id}")
            except Exception:
                try:
                    await handle.signal(DeployWorkflow.approve_deploy)
                    print(f"✅ Sent deploy approval signal to {wf_id}")
                except Exception:
                    await handle.signal(ModelUpdateWorkflow.approve_deploy)
                    print(f"✅ Sent model deploy approval signal to {wf_id}")

        case "status":
            wf_id = sys.argv[2] if len(sys.argv) > 2 else None
            if not wf_id:
                print("Usage: python -m agents.cli status <workflow-id>")
                sys.exit(1)

            handle = client.get_workflow_handle(wf_id)
            desc = await handle.describe()
            print(f"Workflow: {wf_id}")
            print(f"  Status: {desc.status}")
            print(f"  Type:   {desc.workflow_type}")
            print(f"  Started: {desc.start_time}")
            if desc.close_time:
                print(f"  Closed:  {desc.close_time}")

        case _:
            print(f"Unknown command: {command}")
            print(__doc__)
            sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
