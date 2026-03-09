"""
Claude API client for agent workers.
All agents call Claude through this wrapper with domain-specific
system prompts and tool definitions.

Backend is selected via LLM_BACKEND env var:
  anthropic    (default) — Anthropic API
  openai_compat          — OpenAI-compatible API (Moonshot K2, Ollama, etc.)

For openai_compat also set:
  LLM_BASE_URL  e.g. https://api.moonshot.cn/v1  or  http://localhost:11434/v1
  LLM_API_KEY   your key (Ollama accepts any string)
  LLM_MODEL     e.g. kimi-k2  or  qwen2.5-coder:14b
"""

import anthropic
import asyncio
import json
import logging
import subprocess
import os
from pathlib import Path

logger = logging.getLogger(__name__)

WORKSPACE = Path(os.environ.get("WORKSPACE", "/workspace"))
LLM_BACKEND = os.environ.get("LLM_BACKEND", "anthropic")

# Lazy-init clients
_anthropic_client: anthropic.AsyncAnthropic | None = None
_openai_client = None


def _get_anthropic():
    global _anthropic_client
    if _anthropic_client is None:
        _anthropic_client = anthropic.AsyncAnthropic()
    return _anthropic_client


def _get_openai():
    global _openai_client
    if _openai_client is None:
        import openai
        _openai_client = openai.AsyncOpenAI(
            base_url=os.environ["LLM_BASE_URL"],
            api_key=os.environ.get("LLM_API_KEY", "none"),
        )
    return _openai_client


def _to_openai_tools(tools: list[dict]) -> list[dict]:
    """Convert Anthropic tool schema → OpenAI function schema."""
    return [
        {
            "type": "function",
            "function": {
                "name": t["name"],
                "description": t.get("description", ""),
                "parameters": t.get("input_schema", {"type": "object", "properties": {}}),
            },
        }
        for t in tools
    ]


async def call_agent(
    system_prompt: str,
    user_message: str,
    tools: list[dict] | None = None,
    model: str = "claude-sonnet-4-6",
    max_tokens: int = 4096,
    **kwargs,
) -> dict:
    """Core LLM call. Backend selected by LLM_BACKEND env var."""
    if os.environ.get("AGENT_MOCK") == "true":
        print(f"[call_agent] MOCK: {user_message[:80]}", flush=True)
        return _mock_response(system_prompt, user_message)

    if LLM_BACKEND == "openai_compat":
        return await _call_openai_compat(system_prompt, user_message, tools, max_tokens)
    return await _call_anthropic(system_prompt, user_message, tools, model, max_tokens)


async def _call_anthropic(system_prompt, user_message, tools, model, max_tokens) -> dict:
    client = _get_anthropic()
    messages = [{"role": "user", "content": user_message}]
    req = dict(model=model, max_tokens=max_tokens, system=system_prompt, messages=messages)
    if tools:
        req["tools"] = tools

    print(f"[call_agent] anthropic round 0: ~{len(json.dumps(req))} chars", flush=True)
    response = await client.messages.create(**req)
    print(f"[call_agent] done: in={response.usage.input_tokens} out={response.usage.output_tokens}", flush=True)

    tool_round = 0
    while response.stop_reason == "tool_use":
        tool_blocks = [b for b in response.content if b.type == "tool_use"]
        print(f"[call_agent] tool round {tool_round+1}: {[b.name for b in tool_blocks]}", flush=True)

        messages.append({"role": "assistant", "content": response.content})
        results = await asyncio.gather(*[
            asyncio.to_thread(execute_tool, tb.name, tb.input) for tb in tool_blocks
        ])
        messages.append({"role": "user", "content": [
            {"type": "tool_result", "tool_use_id": tb.id, "content": json.dumps(r, default=str)}
            for tb, r in zip(tool_blocks, results)
        ]})

        tool_round += 1
        response = await client.messages.create(**req | {"messages": messages})
        print(f"[call_agent] round {tool_round}: in={response.usage.input_tokens}", flush=True)

    text = "\n".join(b.text for b in response.content if b.type == "text")
    return {"text": text, "usage": {"input_tokens": response.usage.input_tokens, "output_tokens": response.usage.output_tokens}}


async def _call_openai_compat(system_prompt, user_message, tools, max_tokens) -> dict:
    client = _get_openai()
    model = os.environ.get("LLM_MODEL", "kimi-k2")
    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_message},
    ]
    kwargs = dict(model=model, max_tokens=max_tokens, messages=messages)
    if tools:
        kwargs["tools"] = _to_openai_tools(tools)
        kwargs["tool_choice"] = "auto"

    print(f"[call_agent] openai_compat round 0 model={model}", flush=True)
    response = await client.chat.completions.create(**kwargs)
    print(f"[call_agent] done: in={response.usage.prompt_tokens} out={response.usage.completion_tokens}", flush=True)

    tool_round = 0
    while response.choices[0].finish_reason == "tool_calls":
        msg = response.choices[0].message
        tool_calls = msg.tool_calls or []
        print(f"[call_agent] tool round {tool_round+1}: {[tc.function.name for tc in tool_calls]}", flush=True)

        messages.append(msg)
        results = await asyncio.gather(*[
            asyncio.to_thread(execute_tool, tc.function.name, json.loads(tc.function.arguments))
            for tc in tool_calls
        ])
        for tc, r in zip(tool_calls, results):
            messages.append({"role": "tool", "tool_call_id": tc.id, "content": json.dumps(r, default=str)})

        tool_round += 1
        response = await client.chat.completions.create(**kwargs | {"messages": messages})
        print(f"[call_agent] round {tool_round}: in={response.usage.prompt_tokens}", flush=True)

    text = response.choices[0].message.content or ""
    return {"text": text, "usage": {"input_tokens": response.usage.prompt_tokens, "output_tokens": response.usage.completion_tokens}}


def _mock_response(system_prompt: str, user_message: str) -> dict:
    """Return a canned response based on which agent is being called."""
    if "execution plan" in user_message or "JSON" in user_message:
        # analyze_intent mock — return a minimal valid plan
        text = """{
  "summary": "Mock plan: write a README to launch/",
  "safety_critical": false,
  "affected_packages": [],
  "steps": [
    {"agent": "infra", "task_queue": "orchestrator", "action": "Write launch/README.md documenting available launch files", "depends_on": []}
  ]
}"""
    elif "simulation" in user_message.lower() or "SITL" in user_message:
        text = "Mock simulation: all tests passed. No regressions detected."
    elif "review" in user_message.lower() or "DO-178C" in user_message:
        text = "Mock review: pass. No issues found."
    elif "deploy" in user_message.lower():
        text = "Mock deploy: skipped in mock mode."
    elif "documentation" in user_message.lower() or "docs" in user_message.lower():
        text = "Mock docs update: complete."
    elif "write" in user_message.lower() or "create" in user_message.lower():
        # agent-actions mock — write a real file so the test can verify it
        import re as _re
        match = _re.search(r"[\w/.-]+\.(?:md|txt|py|yaml|yml)", user_message)
        rel_path = match.group(0) if match else "ci-test.md"
        path = WORKSPACE / rel_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"# Mock output\n\nGenerated by mock agent.\n")
        text = f"Mock agent: wrote {path}"
    else:
        text = "Mock agent: task complete."

    return {"text": text, "usage": {"input_tokens": 0, "output_tokens": 0}}


def execute_tool(name: str, input_data: dict) -> dict:
    """
    Execute a tool locally. These are the actual operations
    agents can perform on the workspace.
    """
    try:
        match name:
            case "read_file":
                path = WORKSPACE / input_data["path"]
                if not path.exists():
                    return {"error": f"File not found: {input_data['path']}"}
                content = path.read_text()
                if len(content) > 6000:
                    content = content[:6000] + f"\n... [truncated, {len(content)} total chars]"
                return {"content": content, "path": str(path)}

            case "write_file":
                path = WORKSPACE / input_data["path"]
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(input_data["content"])
                return {"status": "written", "path": str(path), "bytes": len(input_data["content"])}

            case "list_directory":
                path = WORKSPACE / input_data.get("path", ".")
                if not path.is_dir():
                    return {"error": f"Not a directory: {path}"}
                entries = []
                for item in sorted(path.iterdir()):
                    if item.name.startswith("."):
                        continue
                    entries.append({
                        "name": item.name,
                        "type": "dir" if item.is_dir() else "file",
                    })
                return {"entries": entries, "path": str(path)}

            case "run_command":
                cmd = input_data["command"]
                cwd = str(WORKSPACE / input_data.get("cwd", "."))
                # Safety: block dangerous commands
                blocked = ["rm -rf /", "dd if=", "mkfs", "> /dev/"]
                if any(b in cmd for b in blocked):
                    return {"error": "Command blocked for safety"}
                result = subprocess.run(
                    cmd, shell=True, cwd=cwd,
                    capture_output=True, text=True, timeout=300,
                )
                return {
                    "stdout": result.stdout[-4000:],  # truncate long output
                    "stderr": result.stderr[-2000:],
                    "returncode": result.returncode,
                }

            case "colcon_build":
                packages = input_data.get("packages", [])
                pkg_flag = f"--packages-select {' '.join(packages)}" if packages else ""
                result = subprocess.run(
                    f"cd {WORKSPACE} && source /opt/ros/humble/setup.bash && colcon build --symlink-install {pkg_flag}",
                    shell=True, executable="/bin/bash",
                    capture_output=True, text=True, timeout=600,
                )
                return {
                    "stdout": result.stdout[-4000:],
                    "stderr": result.stderr[-2000:],
                    "returncode": result.returncode,
                }

            case "run_tests":
                packages = input_data.get("packages", [])
                pkg_flag = f"--packages-select {' '.join(packages)}" if packages else ""
                result = subprocess.run(
                    f"cd {WORKSPACE} && source /opt/ros/humble/setup.bash && colcon test {pkg_flag} && colcon test-result --verbose",
                    shell=True, executable="/bin/bash",
                    capture_output=True, text=True, timeout=600,
                )
                return {
                    "stdout": result.stdout[-4000:],
                    "stderr": result.stderr[-2000:],
                    "returncode": result.returncode,
                }

            case "check_msg_definitions":
                msgs_dir = WORKSPACE / "msgs"
                if not msgs_dir.exists():
                    return {"error": "msgs/ directory not found"}
                msgs = {}
                for ext in ["msg", "srv", "action"]:
                    for f in msgs_dir.rglob(f"*.{ext}"):
                        msgs[str(f.relative_to(WORKSPACE))] = f.read_text()
                return {"definitions": msgs}

            case "git_branch":
                branch = input_data["branch_name"]
                result = subprocess.run(
                    f"git -C {WORKSPACE} checkout -b {branch}",
                    shell=True, capture_output=True, text=True,
                )
                if result.returncode != 0:
                    return {"error": result.stderr.strip()}
                return {"branch": branch, "status": "created"}

            case "git_commit":
                msg = input_data["message"]
                result = subprocess.run(
                    f'git -C {WORKSPACE} add -A && git -C {WORKSPACE} -c user.name="drone-agent" -c user.email="agent@drone.local" commit -m "{msg}"',
                    shell=True, capture_output=True, text=True,
                )
                if result.returncode != 0:
                    return {"error": result.stderr.strip(), "stdout": result.stdout.strip()}
                return {"status": "committed", "output": result.stdout.strip()}

            case "git_push":
                branch_result = subprocess.run(
                    f"git -C {WORKSPACE} rev-parse --abbrev-ref HEAD",
                    shell=True, capture_output=True, text=True,
                )
                branch = branch_result.stdout.strip()
                result = subprocess.run(
                    f"git -C {WORKSPACE} push -u origin {branch}",
                    shell=True, capture_output=True, text=True,
                )
                if result.returncode != 0:
                    return {"error": result.stderr.strip()}
                return {"status": "pushed", "branch": branch}

            case _:
                return {"error": f"Unknown tool: {name}"}

    except subprocess.TimeoutExpired:
        return {"error": f"Command timed out after 300s"}
    except Exception as e:
        return {"error": str(e)}
