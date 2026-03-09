"""
Unit tests for llm_client — run without API keys or GPU.
All tests operate in mock mode or test pure logic only.
"""

import json
import os
import pytest

os.environ.setdefault("AGENT_MOCK", "true")
os.environ.setdefault("WORKSPACE", "/tmp/test-workspace")


from agents.shared.llm_client import (
    _mock_response,
    _to_openai_tools,
    execute_tool,
    WORKSPACE,
)


# ── _to_openai_tools ────────────────────────────

def test_to_openai_tools_structure():
    anthropic_tools = [
        {
            "name": "read_file",
            "description": "Read a file",
            "input_schema": {
                "type": "object",
                "properties": {"path": {"type": "string"}},
                "required": ["path"],
            },
        }
    ]
    result = _to_openai_tools(anthropic_tools)
    assert len(result) == 1
    assert result[0]["type"] == "function"
    assert result[0]["function"]["name"] == "read_file"
    assert result[0]["function"]["description"] == "Read a file"
    assert result[0]["function"]["parameters"]["properties"]["path"]["type"] == "string"


def test_to_openai_tools_missing_description():
    tools = [{"name": "git_push", "input_schema": {"type": "object", "properties": {}}}]
    result = _to_openai_tools(tools)
    assert result[0]["function"]["description"] == ""


def test_to_openai_tools_empty():
    assert _to_openai_tools([]) == []


# ── _mock_response ───────────────────────────────

def test_mock_response_plan():
    result = _mock_response("", "Analyze this request and return a JSON execution plan.")
    assert result["usage"] == {"input_tokens": 0, "output_tokens": 0}
    plan = json.loads(result["text"])
    assert "summary" in plan
    assert "steps" in plan
    assert isinstance(plan["steps"], list)
    assert plan["steps"][0]["agent"] in {
        "perception-dev", "nav-dev", "control-dev", "autonomy-dev",
        "comms-dev", "safety-dev", "infra", "code-review",
        "sim-test", "ml-pipeline", "deploy",
    }


def test_mock_response_simulation():
    result = _mock_response("", "Run simulation tests for this plan")
    assert "pass" in result["text"].lower() or "mock" in result["text"].lower()


def test_mock_response_review():
    result = _mock_response("", "Review the changes made. DO-178C checklist.")
    assert "pass" in result["text"].lower()


def test_mock_response_deploy():
    result = _mock_response("", "Deploy version 1.0 to orin.")
    assert "mock" in result["text"].lower()


def test_mock_response_docs():
    result = _mock_response("", "Update documentation for changes.")
    assert "mock" in result["text"].lower()


def test_mock_response_domain_agent_writes_file(tmp_path, monkeypatch):
    monkeypatch.setattr("agents.shared.llm_client.WORKSPACE", tmp_path)
    result = _mock_response("You are an infra agent.", "Perform this task in the workspace: write a config file")
    assert "mock" in result["text"].lower()
    assert (tmp_path / "launch" / "README.md").exists()


# ── execute_tool ─────────────────────────────────

def test_execute_tool_read_file(tmp_path, monkeypatch):
    monkeypatch.setattr("agents.shared.llm_client.WORKSPACE", tmp_path)
    (tmp_path / "hello.txt").write_text("hello world")
    result = execute_tool("read_file", {"path": "hello.txt"})
    assert result["content"] == "hello world"


def test_execute_tool_read_file_not_found(tmp_path, monkeypatch):
    monkeypatch.setattr("agents.shared.llm_client.WORKSPACE", tmp_path)
    result = execute_tool("read_file", {"path": "missing.txt"})
    assert "error" in result


def test_execute_tool_read_file_truncates(tmp_path, monkeypatch):
    monkeypatch.setattr("agents.shared.llm_client.WORKSPACE", tmp_path)
    (tmp_path / "big.txt").write_text("x" * 10000)
    result = execute_tool("read_file", {"path": "big.txt"})
    assert len(result["content"]) < 10000
    assert "truncated" in result["content"]


def test_execute_tool_write_file(tmp_path, monkeypatch):
    monkeypatch.setattr("agents.shared.llm_client.WORKSPACE", tmp_path)
    result = execute_tool("write_file", {"path": "out/test.txt", "content": "hello"})
    assert result["status"] == "written"
    assert (tmp_path / "out" / "test.txt").read_text() == "hello"


def test_execute_tool_list_directory(tmp_path, monkeypatch):
    monkeypatch.setattr("agents.shared.llm_client.WORKSPACE", tmp_path)
    (tmp_path / "a.txt").write_text("")
    (tmp_path / "subdir").mkdir()
    result = execute_tool("list_directory", {"path": "."})
    names = [e["name"] for e in result["entries"]]
    assert "a.txt" in names
    assert "subdir" in names


def test_execute_tool_list_directory_not_found(tmp_path, monkeypatch):
    monkeypatch.setattr("agents.shared.llm_client.WORKSPACE", tmp_path)
    result = execute_tool("list_directory", {"path": "nonexistent"})
    assert "error" in result


def test_execute_tool_run_command_blocked():
    result = execute_tool("run_command", {"command": "rm -rf /"})
    assert "error" in result
    assert "blocked" in result["error"].lower()


def test_execute_tool_run_command(tmp_path, monkeypatch):
    monkeypatch.setattr("agents.shared.llm_client.WORKSPACE", tmp_path)
    result = execute_tool("run_command", {"command": "echo hello"})
    assert result["returncode"] == 0
    assert "hello" in result["stdout"]


def test_execute_tool_unknown():
    result = execute_tool("nonexistent_tool", {})
    assert "error" in result
    assert "Unknown tool" in result["error"]


# ── call_agent mock mode ─────────────────────────

@pytest.mark.asyncio
async def test_call_agent_mock_mode():
    """call_agent returns mock response without hitting any API."""
    from agents.shared.llm_client import call_agent
    result = await call_agent(
        system_prompt="You are an infra agent.",
        user_message="Analyze this request and return a JSON execution plan. REQUEST: test",
    )
    assert "text" in result
    assert "usage" in result
    assert result["usage"]["input_tokens"] == 0
