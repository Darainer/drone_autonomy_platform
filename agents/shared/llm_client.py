"""
Claude API client for agent workers.
All agents call Claude through this wrapper with domain-specific
system prompts and tool definitions.
"""

import anthropic
import json
import subprocess
import os
from pathlib import Path
from typing import Any

client = anthropic.Anthropic()  # reads ANTHROPIC_API_KEY from env

WORKSPACE = Path(os.environ.get("WORKSPACE", "/workspace"))


def call_agent(
    system_prompt: str,
    user_message: str,
    tools: list[dict] | None = None,
    model: str = "claude-sonnet-4-20250514",
    max_tokens: int = 8192,
) -> dict:
    """
    Core LLM call used by all agent workers.
    Handles tool-use loops automatically.
    """
    messages = [{"role": "user", "content": user_message}]

    kwargs = dict(
        model=model,
        max_tokens=max_tokens,
        system=system_prompt,
        messages=messages,
    )
    if tools:
        kwargs["tools"] = tools

    response = client.messages.create(**kwargs)

    # Tool-use loop: Claude calls tools → we execute → feed back results
    while response.stop_reason == "tool_use":
        tool_blocks = [b for b in response.content if b.type == "tool_use"]

        messages.append({"role": "assistant", "content": response.content})

        tool_results = []
        for tb in tool_blocks:
            result = execute_tool(tb.name, tb.input)
            tool_results.append({
                "type": "tool_result",
                "tool_use_id": tb.id,
                "content": json.dumps(result, default=str),
            })

        messages.append({"role": "user", "content": tool_results})

        response = client.messages.create(**kwargs | {"messages": messages})

    # Extract final text
    text = "\n".join(b.text for b in response.content if b.type == "text")

    return {
        "text": text,
        "usage": {
            "input_tokens": response.usage.input_tokens,
            "output_tokens": response.usage.output_tokens,
        },
    }


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

            case _:
                return {"error": f"Unknown tool: {name}"}

    except subprocess.TimeoutExpired:
        return {"error": f"Command timed out after 300s"}
    except Exception as e:
        return {"error": str(e)}
