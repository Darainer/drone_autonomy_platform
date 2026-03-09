"""
Tests for analyze_intent plan validation logic —
ensures hallucinated agent IDs and queues are corrected.
"""

import json
import os
import pytest

os.environ.setdefault("AGENT_MOCK", "true")
os.environ.setdefault("WORKSPACE", "/tmp/test-workspace")


VALID_AGENTS = {
    "perception-dev", "nav-dev", "control-dev", "autonomy-dev",
    "comms-dev", "safety-dev", "infra", "code-review",
    "sim-test", "ml-pipeline", "deploy",
}

AGENT_TO_QUEUE = {
    "perception-dev": "ros2-dev", "nav-dev": "ros2-dev", "control-dev": "ros2-dev",
    "autonomy-dev": "ros2-dev", "comms-dev": "ros2-dev", "safety-dev": "ros2-dev",
    "infra": "orchestrator", "code-review": "orchestrator",
    "sim-test": "simulation", "ml-pipeline": "ml-pipeline", "deploy": "deployment",
}


def apply_validation(plan: dict) -> dict:
    """Mirror of the validation block in analyze_intent."""
    for step in plan.get("steps", []):
        if step.get("agent") not in VALID_AGENTS:
            step["agent"] = "infra"
        step["task_queue"] = AGENT_TO_QUEUE[step["agent"]]
    return plan


def test_valid_plan_passes_unchanged():
    plan = {
        "summary": "Add a perception node",
        "safety_critical": False,
        "steps": [{"agent": "perception-dev", "task_queue": "ros2-dev", "action": "do it", "depends_on": []}],
    }
    result = apply_validation(plan)
    assert result["steps"][0]["agent"] == "perception-dev"
    assert result["steps"][0]["task_queue"] == "ros2-dev"


def test_hallucinated_agent_corrected_to_infra():
    plan = {
        "summary": "test",
        "steps": [{"agent": "docs-agent", "task_queue": "docs-writes", "action": "write docs"}],
    }
    result = apply_validation(plan)
    assert result["steps"][0]["agent"] == "infra"
    assert result["steps"][0]["task_queue"] == "orchestrator"


def test_wrong_queue_corrected():
    plan = {
        "summary": "test",
        "steps": [{"agent": "nav-dev", "task_queue": "wrong-queue", "action": "plan path"}],
    }
    result = apply_validation(plan)
    assert result["steps"][0]["task_queue"] == "ros2-dev"


def test_all_valid_agents_get_correct_queues():
    for agent, expected_queue in AGENT_TO_QUEUE.items():
        plan = {"steps": [{"agent": agent, "task_queue": "wrong", "action": "test"}]}
        result = apply_validation(plan)
        assert result["steps"][0]["task_queue"] == expected_queue, f"Failed for agent {agent}"


def test_empty_steps():
    plan = {"summary": "nothing", "steps": []}
    result = apply_validation(plan)
    assert result["steps"] == []


def test_mock_plan_output_is_valid():
    """The mock response plan must pass validation without corrections."""
    from agents.shared.llm_client import _mock_response
    result = _mock_response("", "Analyze this request and return a JSON execution plan. REQUEST: test")
    plan = json.loads(result["text"])
    for step in plan["steps"]:
        assert step["agent"] in VALID_AGENTS, f"Invalid agent in mock plan: {step['agent']}"
        assert step["task_queue"] == AGENT_TO_QUEUE[step["agent"]]
