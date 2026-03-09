# Contributing

## Setup

```bash
# Build with your host UID/GID — prevents root-owned files in the workspace
cd docker
UID=$(id -u) GID=$(id -g) docker compose build
docker compose up -d
```

See `agents/README.md` for full agent system setup including local LLM backends.

## Safety-Critical Code Paths

Changes to the following packages require a safety review before merge:

- `src/control/` — flight control algorithms
- `src/safety/` — failsafe and emergency handling
- `src/navigation/` — path planning

The agent workflow automatically detects these paths and applies a DO-178C review
checklist plus a mandatory human approval gate before any deploy.

## Pull Request Process

1. Fork the repository and create a feature branch off `main`
2. Make your changes inside the Docker environment to ensure they build cleanly
3. Run the smoke test locally before pushing:
   ```bash
   docker build -t drone_autonomy_platform .
   docker run --rm drone_autonomy_platform \
     bash /ws/src/drone_autonomy_platform/scripts/smoke_test.sh
   ```
4. Submit a PR with a description of what changed and why
5. CI will build the image and run the smoke test automatically

Agent-generated PRs are created automatically by the workflow — review the
Temporal UI for execution history and per-step logs before merging.

## Adding a New Node

1. Create `src/<name>/` with `CMakeLists.txt`, `package.xml`, `src/<name>_node.cpp`, and `launch/<name>.launch.py`
2. Add an `install(TARGETS <name>_node DESTINATION lib/${PROJECT_NAME})` rule in `CMakeLists.txt`
3. Add the package to `launch/platform_core.launch.py` (or `platform.launch.py` if it requires Isaac ROS)
4. Add the expected node name to the `smoke_test.sh` check list

## Perception / Isaac ROS

The `perception` package requires the NVIDIA Isaac ROS apt registry and is only buildable on
Jetson hardware. Do not add Isaac ROS dependencies to any other package.
