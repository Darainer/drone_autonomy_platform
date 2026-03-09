"""
Temporal activities for the orchestrator and agent dispatch.

Activities are the actual units of work. Each one calls Claude
with the appropriate system prompt and tools for the task.
"""

import json
import subprocess
import re
from temporalio import activity

from agents.shared.llm_client import call_agent, WORKSPACE
from agents.shared.prompts import get_prompt, ORCHESTRATOR, CODE_REVIEW
from agents.shared.tools import (
    ORCHESTRATOR_TOOLS,
    DOMAIN_DEV_TOOLS,
    SIM_TOOLS,
    ML_TOOLS,
    DEPLOY_TOOLS,
    REVIEW_TOOLS,
)


TOOL_SETS = {
    "orchestrator": ORCHESTRATOR_TOOLS,
    "perception-dev": DOMAIN_DEV_TOOLS,
    "nav-dev": DOMAIN_DEV_TOOLS,
    "control-dev": DOMAIN_DEV_TOOLS,
    "autonomy-dev": DOMAIN_DEV_TOOLS,
    "comms-dev": DOMAIN_DEV_TOOLS,
    "safety-dev": DOMAIN_DEV_TOOLS,
    "sim-test": SIM_TOOLS,
    "ml-pipeline": ML_TOOLS,
    "deploy": DEPLOY_TOOLS,
    "code-review": REVIEW_TOOLS,
    "infra": DOMAIN_DEV_TOOLS,
}


@activity.defn
async def create_feature_branch(description: str) -> dict:
    """
    Create a git feature branch for the workflow.
    Branch name is derived from the feature description.
    """
    slug = re.sub(r"[^a-z0-9]+", "-", description.lower())[:50].strip("-")
    branch = f"feature/{slug}"

    create_result = subprocess.run(
        f"git -C {WORKSPACE} checkout -b {branch}",
        shell=True, capture_output=True, text=True,
    )
    if create_result.returncode != 0:
        # Branch may already exist — try to check it out
        checkout_result = subprocess.run(
            f"git -C {WORKSPACE} checkout {branch}",
            shell=True, capture_output=True, text=True,
        )
        if checkout_result.returncode != 0:
            # Detached HEAD (CI) or other git state — log and continue without branching
            activity.logger.warning(f"Could not create/checkout branch {branch}: {checkout_result.stderr.strip()}")
            return {"branch": "detached", "warning": checkout_result.stderr.strip()}

    activity.logger.info(f"On branch: {branch}")
    return {"branch": branch}


@activity.defn
async def analyze_intent(description: str) -> dict:
    """
    Orchestrator analyzes user intent and produces a structured plan.
    Returns JSON with steps, affected packages, safety classification.
    """
    result = await call_agent(
        system_prompt=ORCHESTRATOR,
        user_message=f"""Analyze this request and return a JSON execution plan.

REQUEST: {description}

Return ONLY a JSON object — no prose, no markdown fences:
{{
  "summary": "one-line description of what will be done",
  "safety_critical": false,
  "affected_packages": [],
  "steps": [
    {{"agent": "agent-id", "task_queue": "queue-name", "action": "concise task description", "depends_on": []}}
  ]
}}

Keep each step action under 2 sentences. Use only the valid agents and queues listed above.""",
        tools=None,
    )

    # Parse the JSON plan from Claude's response
    text = result["text"]
    try:
        # Try to find JSON in the response
        start = text.index("{")
        end = text.rindex("}") + 1
        plan = json.loads(text[start:end])
    except (ValueError, json.JSONDecodeError):
        # Fallback: return as unstructured
        plan = {
            "summary": text,
            "safety_critical": False,
            "steps": [{"agent": "perception-dev", "task_queue": "ros2-dev", "action": description}],
        }

    # Enforce valid agents and task queues — correct any hallucinations
    valid_agents = {
        "perception-dev", "nav-dev", "control-dev", "autonomy-dev",
        "comms-dev", "safety-dev", "infra", "code-review",
        "sim-test", "ml-pipeline", "deploy",
    }
    agent_to_queue = {
        "perception-dev": "ros2-dev", "nav-dev": "ros2-dev", "control-dev": "ros2-dev",
        "autonomy-dev": "ros2-dev", "comms-dev": "ros2-dev", "safety-dev": "ros2-dev",
        "infra": "orchestrator", "code-review": "orchestrator",
        "sim-test": "simulation", "ml-pipeline": "ml-pipeline", "deploy": "deployment",
    }
    for step in plan.get("steps", []):
        if step.get("agent") not in valid_agents:
            step["agent"] = "infra"
        step["task_queue"] = agent_to_queue[step["agent"]]

    activity.logger.info(f"Plan: {plan.get('summary', 'no summary')}")
    return plan


@activity.defn
async def run_domain_agent(agent_id: str, action: str, context: str) -> dict:
    """
    Dispatch work to a domain-specific agent.
    The agent gets its system prompt and tools, plus the action to perform.
    """
    system_prompt = get_prompt(agent_id)
    tools = TOOL_SETS.get(agent_id, DOMAIN_DEV_TOOLS)

    result = await call_agent(
        system_prompt=system_prompt,
        user_message=f"""Perform this task in the workspace:

TASK: {action}

CONTEXT: {context}

Steps:
1. First, read the relevant existing files to understand current code
2. Implement the changes
3. Build to verify no compilation errors
4. Run existing tests to verify no regressions
5. Write new tests if adding new functionality

Report what you did and any issues encountered.""",
        tools=tools,
    )

    activity.logger.info(f"Agent {agent_id} completed: {result['text'][:200]}")
    return {
        "agent": agent_id,
        "action": action,
        "result": result["text"],
        "tokens": result["usage"],
    }


@activity.defn
async def run_code_review(plan: dict, results: list) -> dict:
    """
    Run automated code review on changes.
    Safety-critical paths get extra scrutiny.
    """
    safety_critical = plan.get("safety_critical", False)
    packages = plan.get("affected_packages", [])

    result = await call_agent(
        system_prompt=CODE_REVIEW,
        user_message=f"""Review the changes made to these packages: {packages}

Safety-critical: {safety_critical}

Changes summary:
{json.dumps([r.get('result', '')[:500] for r in results], indent=2)}

Steps:
1. Read the modified files
2. Run linters (clang-tidy for C++, pylint for Python)
3. Check test coverage
4. If safety-critical: apply DO-178C checklist
5. Return pass/fail with issues list""",
        tools=REVIEW_TOOLS,
    )

    # Determine pass/fail from Claude's assessment
    text = result["text"].lower()
    passed = "pass" in text and "fail" not in text and "block" not in text

    return {
        "passed": passed,
        "safety_critical": safety_critical,
        "review": result["text"],
        "tokens": result["usage"],
    }


@activity.defn
async def run_simulation(plan: dict) -> dict:
    """
    Run simulation tests relevant to the changes.
    """
    result = await call_agent(
        system_prompt=get_prompt("sim-test"),
        user_message=f"""Run simulation tests for this plan:
{json.dumps(plan, indent=2)}

Steps:
1. Check which launch files are relevant
2. Run unit tests for affected packages
3. If SITL test is needed, set up the simulation scenario
4. Collect and summarize results""",
        tools=SIM_TOOLS,
    )

    return {
        "result": result["text"],
        "tokens": result["usage"],
    }


@activity.defn
async def run_deploy(version: str, packages: list | None, target: str) -> dict:
    """
    Cross-compile and deploy to target hardware.
    """
    result = await call_agent(
        system_prompt=get_prompt("deploy"),
        user_message=f"""Deploy version {version} to {target}.
Packages: {packages or 'all'}

Steps:
1. Verify build succeeds on x86
2. Build Docker image for linux/arm64
3. Deploy to target via SSH
4. Run health check
5. Report status""",
        tools=DEPLOY_TOOLS,
    )

    return {
        "version": version,
        "target": target,
        "result": result["text"],
        "tokens": result["usage"],
    }


@activity.defn
async def update_docs(plan: dict, results: list) -> dict:
    """
    Update documentation based on changes made.
    """
    result = await call_agent(
        system_prompt=get_prompt("infra"),
        user_message=f"""Update documentation for changes:

Plan: {plan.get('summary', '')}
Affected packages: {plan.get('affected_packages', [])}

Update relevant docs in docs/ and any READMEs.
If new message types were added to msgs/, document them.""",
        tools=DOMAIN_DEV_TOOLS,
    )

    return {"result": result["text"], "tokens": result["usage"]}
