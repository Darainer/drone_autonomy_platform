---
name: c4
description: Generate or refresh C4-model architecture views (context, container, component, topic inventory) from the ROS2 source code using scripts/generate_c4.py. Use when asked for architecture diagrams or C4 views, after adding/removing nodes or topics, when diagrams have drifted from code, or to inspect the topic graph.
---

# C4 Architecture Views from Code

`scripts/generate_c4.py` parses `src/**` for rclcpp/rclpy constructs
(`create_publisher`, `create_subscription`, `create_client`, `create_service`,
`message_filters.Subscriber`, node names, parameter-default topics), applies
`remappings=` from launch-file `Node(...)` actions (matched by node name), and
emits **C4-PlantUML** diagram sources rendered to SVG with Graphviz into
`docs/architecture/c4/`:

| File | View |
|---|---|
| `level1_context.{puml,svg,md}` | L1 system context (operator, GCS, PX4, sensors) |
| `level2_container.{puml,svg,md}` | L2 — every ROS2 node grouped by package, all topic/service flows |
| `level3_component_<pkg>.{puml,svg,md}` | L3 per package + interface table |
| `topics.md` | Topic/service inventory with link status, remap warnings, dangling interfaces |

The `.md` page embeds the `.svg`; the `.puml` is the diagram source. All
three are generated and committed together. Dashed edges = *needs remap*
candidates (basename match only); dotted edges = service calls.

## Tooling

Rendering needs Java + Graphviz + the PlantUML jar (pinned, from Maven
Central). One-time setup per machine/container:

```bash
bash scripts/setup_c4_tooling.sh   # installs graphviz + plantuml jar
```

The jar is just a file, so it's fetched to a user-writable location
(`~/.local/share/plantuml/plantuml.jar`, no root needed). Only
`apt-get install graphviz` needs sudo, and only if `dot` isn't already on
PATH. The generator finds PlantUML via, in order: `$PLANTUML_JAR`,
`plantuml` on PATH, the user-writable default, `/opt/plantuml/plantuml.jar`
(legacy fallback for prebuilt container images). SVGs are rendered with
`-nometadata`, so output is byte-stable for a given PlantUML + Graphviz
version — keep the pinned version in `scripts/setup_c4_tooling.sh` in sync
everywhere (CI, containers), otherwise `--check` can report false SVG drift.

## Commands

```bash
python scripts/generate_c4.py              # regenerate all views + SVGs
python scripts/generate_c4.py --check      # exit 1 if views don't match the code (CI/drift)
python scripts/generate_c4.py --no-render  # .puml/.md only, when Java/Graphviz unavailable
```

`--check` without tooling degrades gracefully: it verifies `.md`/`.puml`
content and that SVGs exist, but not SVG content.

Generated files carry a `GENERATED FILE` header — **never hand-edit them**
(including the `.puml` and `.svg`); fix the source or the generator instead.

## Procedure

1. Run the generator; skim its output for parse warnings.
2. **Read `topics.md` critically** — it encodes real integration state:
   - `needs remap`: publisher and subscriber use same-basename private (`~/`)
     topics on different nodes. They only connect if a launch file remaps
     them. Fix the launch file (or the topic names), don't ignore it.
   - `dangling`: interface with no counterpart in-repo. Either it's consumed
     externally (add a prefix to `EXTERNAL_SYSTEMS` in the script) or it's
     dead/unfinished wiring.
3. **Look at the rendered diagram** (`.svg`, or render the `.puml` to PNG)
   after significant topology changes — an unreadable diagram usually means
   a mis-parsed interface, not just bad luck with layout.
4. Commit regenerated views (`.md` + `.puml` + `.svg`) **in the same
   commit** as the node/topic change.

## Extending the generator (edit `scripts/generate_c4.py`)

- **New external system** (new driver, GCS bridge, Isaac ROS graph): add an
  entry to `EXTERNAL_SYSTEMS` — `(alias, name, description, topic-prefix regex)`.
  Context-level actors (operator, GCS) live in the `render_context()` template.
- **New API patterns** (actions, lifecycle nodes, composed nodes): add a regex
  next to the existing `CPP_*`/`PY_*` patterns and wire it into
  `parse_sources()`.
- **Diagram styling** (line styles, legend, layout): the C4-PlantUML calls
  are emitted in `render_container()` / `render_component()` and the
  `REL_TAGS` list — use C4-PlantUML macros (`AddRelTag`, `LAYOUT_*`,
  `SHOW_LEGEND`), not raw PlantUML skinparams, so the C4 semantics stay
  intact.
- If a topic name is computed at runtime and can't be parsed, the node's
  "Parser notes" section in its L3 view will say so — prefer declaring the
  topic as a parameter default (`declare_parameter("image_topic", "/oak/...")`),
  which the parser resolves.

## When to run

- After any change to node pub/sub/service code or launch remappings
  (the `architecture` and `design` skills reference this).
- `--check` belongs in CI and pre-merge review of any `src/` change
  (install tooling with `scripts/setup_c4_tooling.sh` first, or run with
  `--no-render` for the text-only gate).
