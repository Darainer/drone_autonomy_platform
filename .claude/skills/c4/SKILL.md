---
name: c4
description: Generate or refresh C4-model architecture views (context, container, component, topic inventory) from the ROS2 source code using scripts/generate_c4.py. Use when asked for architecture diagrams or C4 views, after adding/removing nodes or topics, when diagrams have drifted from code, or to inspect the topic graph.
---

# C4 Architecture Views from Code

`scripts/generate_c4.py` parses `src/**` for rclcpp/rclpy constructs
(`create_publisher`, `create_subscription`, `create_client`, `create_service`,
`message_filters.Subscriber`, node names, parameter-default topics) and emits
Mermaid C4 diagrams to `docs/architecture/c4/`:

| File | View |
|---|---|
| `level1_context.md` | L1 system context (operator, GCS, PX4, sensors) |
| `level2_container.md` | L2 — every ROS2 node grouped by package, all topic/service flows |
| `level3_component_<pkg>.md` | L3 per package + interface table |
| `topics.md` | Topic/service inventory with link status, remap warnings, dangling interfaces |

## Commands

```bash
python scripts/generate_c4.py          # regenerate all views
python scripts/generate_c4.py --check  # exit 1 if views don't match the code (CI/drift)
```

Generated files carry a `GENERATED FILE` header — **never hand-edit them**;
fix the source or the generator instead.

## Procedure

1. Run the generator; skim its output for parse warnings.
2. **Read `topics.md` critically** — it encodes real integration state:
   - `needs remap`: publisher and subscriber use same-basename private (`~/`)
     topics on different nodes. They only connect if a launch file remaps
     them. Fix the launch file (or the topic names), don't ignore it.
   - `dangling`: interface with no counterpart in-repo. Either it's consumed
     externally (add a prefix to `EXTERNAL_SYSTEMS` in the script) or it's
     dead/unfinished wiring.
3. Commit regenerated views **in the same commit** as the node/topic change.

## Extending the generator (edit `scripts/generate_c4.py`)

- **New external system** (new driver, GCS bridge, Isaac ROS graph): add an
  entry to `EXTERNAL_SYSTEMS` — `(alias, name, description, topic-prefix regex)`.
  Context-level actors (operator, GCS) live in the `render_context()` template.
- **New API patterns** (actions, lifecycle nodes, composed nodes): add a regex
  next to the existing `CPP_*`/`PY_*` patterns and wire it into
  `parse_sources()`.
- If a topic name is computed at runtime and can't be parsed, the node's
  "Parser notes" section in its L3 view will say so — prefer declaring the
  topic as a parameter default (`declare_parameter("image_topic", "/oak/...")`),
  which the parser resolves.

## When to run

- After any change to node pub/sub/service code or launch remappings
  (the `architecture` and `design` skills reference this).
- `--check` belongs in CI and pre-merge review of any `src/` change.
