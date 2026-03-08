"""
Tool definitions for Claude function calling.
Each agent picks the tools relevant to its domain.
"""

# ── File Operations ──────────────────────────────

READ_FILE = {
    "name": "read_file",
    "description": "Read the contents of a file in the workspace",
    "input_schema": {
        "type": "object",
        "properties": {
            "path": {
                "type": "string",
                "description": "Path relative to workspace root (e.g. 'src/perception/src/detector.cpp')",
            },
        },
        "required": ["path"],
    },
}

WRITE_FILE = {
    "name": "write_file",
    "description": "Write content to a file (creates parent directories automatically)",
    "input_schema": {
        "type": "object",
        "properties": {
            "path": {
                "type": "string",
                "description": "Path relative to workspace root",
            },
            "content": {
                "type": "string",
                "description": "Full file content to write",
            },
        },
        "required": ["path", "content"],
    },
}

LIST_DIRECTORY = {
    "name": "list_directory",
    "description": "List files and directories at a path",
    "input_schema": {
        "type": "object",
        "properties": {
            "path": {
                "type": "string",
                "description": "Path relative to workspace root (default: root)",
                "default": ".",
            },
        },
    },
}

# ── Build & Test ─────────────────────────────────

COLCON_BUILD = {
    "name": "colcon_build",
    "description": "Build ROS2 packages with colcon. Leave packages empty to build all.",
    "input_schema": {
        "type": "object",
        "properties": {
            "packages": {
                "type": "array",
                "items": {"type": "string"},
                "description": "Package names to build (empty = build all)",
                "default": [],
            },
        },
    },
}

RUN_TESTS = {
    "name": "run_tests",
    "description": "Run tests for ROS2 packages",
    "input_schema": {
        "type": "object",
        "properties": {
            "packages": {
                "type": "array",
                "items": {"type": "string"},
                "description": "Package names to test (empty = test all)",
                "default": [],
            },
        },
    },
}

RUN_COMMAND = {
    "name": "run_command",
    "description": "Run a shell command in the workspace",
    "input_schema": {
        "type": "object",
        "properties": {
            "command": {
                "type": "string",
                "description": "Shell command to execute",
            },
            "cwd": {
                "type": "string",
                "description": "Working directory relative to workspace (default: root)",
                "default": ".",
            },
        },
        "required": ["command"],
    },
}

# ── ROS2-Specific ────────────────────────────────

CHECK_MSG_DEFINITIONS = {
    "name": "check_msg_definitions",
    "description": "List all custom message, service, and action definitions in msgs/",
    "input_schema": {
        "type": "object",
        "properties": {},
    },
}


# ── Git Operations ───────────────────────────────

GIT_BRANCH = {
    "name": "git_branch",
    "description": "Create and checkout a new git branch from current HEAD",
    "input_schema": {
        "type": "object",
        "properties": {
            "branch_name": {
                "type": "string",
                "description": "Branch name to create (e.g. 'feature/add-gps-node')",
            },
        },
        "required": ["branch_name"],
    },
}

GIT_COMMIT = {
    "name": "git_commit",
    "description": "Stage all changes and create a git commit",
    "input_schema": {
        "type": "object",
        "properties": {
            "message": {
                "type": "string",
                "description": "Commit message",
            },
        },
        "required": ["message"],
    },
}

GIT_PUSH = {
    "name": "git_push",
    "description": "Push the current branch to origin",
    "input_schema": {
        "type": "object",
        "properties": {},
    },
}


# ── Agent Tool Sets ──────────────────────────────
# Each agent gets a curated subset of tools

ORCHESTRATOR_TOOLS = [READ_FILE, LIST_DIRECTORY, RUN_COMMAND, GIT_BRANCH]

DOMAIN_DEV_TOOLS = [READ_FILE, WRITE_FILE, LIST_DIRECTORY, COLCON_BUILD, RUN_TESTS, CHECK_MSG_DEFINITIONS, GIT_COMMIT]

SIM_TOOLS = [READ_FILE, WRITE_FILE, LIST_DIRECTORY, RUN_COMMAND, COLCON_BUILD, RUN_TESTS, GIT_COMMIT]

ML_TOOLS = [READ_FILE, WRITE_FILE, LIST_DIRECTORY, RUN_COMMAND, GIT_COMMIT]

DEPLOY_TOOLS = [READ_FILE, LIST_DIRECTORY, RUN_COMMAND]

REVIEW_TOOLS = [READ_FILE, LIST_DIRECTORY, RUN_COMMAND, RUN_TESTS, GIT_PUSH]
