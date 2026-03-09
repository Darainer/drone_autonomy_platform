#!/usr/bin/env python3
"""
Submit a task to the agent workforce from Claude Code.

Usage:
    python scripts/submit_task.py "description" [--plan plan.json] [--rework "feedback"]

Claude Code calls this directly after forming a plan. The script submits the
workflow, waits for completion, and prints a structured result for review.
"""

import argparse
import asyncio
import json
import os
import sys
import uuid
from datetime import datetime

from temporalio.client import Client

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from agents.orchestrator.workflows import NewFeatureWorkflow, FeatureRequest


async def submit(description: str, plan: dict | None, rework_context: str) -> dict:
    temporal_address = os.environ.get("TEMPORAL_ADDRESS", "localhost:7233")
    client = await Client.connect(temporal_address)

    workflow_id = f"task-{datetime.now().strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:6]}"

    print(f"[submit_task] Submitting workflow {workflow_id}", flush=True)
    if plan:
        print(f"[submit_task] Plan: {plan.get('summary', '')} ({len(plan.get('steps', []))} steps)", flush=True)

    handle = await client.start_workflow(
        NewFeatureWorkflow.run,
        FeatureRequest(
            description=description,
            auto_approve=True,
            plan=plan,
            rework_context=rework_context,
        ),
        id=workflow_id,
        task_queue="orchestrator",
    )

    print(f"[submit_task] Waiting for result... (Temporal UI: http://localhost:8080/namespaces/default/workflows/{workflow_id})", flush=True)

    result = await handle.result()
    return {"workflow_id": workflow_id, **result}


def main():
    parser = argparse.ArgumentParser(description="Submit a task to the drone agent workforce")
    parser.add_argument("description", help="Natural language task description")
    parser.add_argument("--plan", help="Path to pre-formed plan JSON file, or inline JSON string")
    parser.add_argument("--rework", default="", help="Feedback from a previous failed attempt")
    args = parser.parse_args()

    plan = None
    if args.plan:
        if args.plan.strip().startswith("{"):
            plan = json.loads(args.plan)
        else:
            with open(args.plan) as f:
                plan = json.load(f)

    result = asyncio.run(submit(args.description, plan, args.rework))

    print("\n" + "═" * 60)
    print("WORKFLOW RESULT")
    print("═" * 60)
    print(json.dumps(result, indent=2, default=str))

    # Exit non-zero if review failed — lets Claude Code detect failure
    review = result.get("review", {})
    if not review.get("passed", True):
        sys.exit(1)


if __name__ == "__main__":
    main()
