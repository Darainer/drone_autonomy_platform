---
name: architecture
description: Create or update architecture documentation under docs/architecture/ — subsystem architectures, use cases, hardware/integration docs — and keep generated C4 views consistent with the code. Use when documenting a subsystem, adding a use case, or when architecture docs have drifted from the source.
---

# Architecture Documentation Workflow

`docs/architecture/` layout:

| Path | Content | Maintained |
|---|---|---|
| `c4/` | C4 model views (context/container/component) + topic inventory | **Generated** — `python scripts/generate_c4.py`, never hand-edit |
| `use_cases/*.md` | Mission-level use cases (surveying, agriculture, SAR, …) | By hand |
| `*.md` | Subsystem architectures (perception, telemetry, hardware, PX4) | By hand |

## Rules

1. **Structure comes from the generator.** Node/topic/package structure is
   shown by linking the generated C4 views (`c4/level2_container.md`,
   `c4/level3_component_<pkg>.md`, `c4/topics.md`). Don't hand-draw
   node/topic box diagrams in subsystem docs — they drift. Hand-drawn diagrams
   are fine for *internals* the generator can't see (model architectures,
   state machines, timing diagrams); prefer Mermaid over ASCII art for new ones.
2. **Requirements live elsewhere.** Reference UIDs from
   `docs/requirements/*.sdoc` (e.g. "meets COMP-3"); never define new
   requirement IDs in an architecture doc — invoke the `requirements` skill.
3. **Standard subsystem doc structure** (see
   `perception_architecture.md` as the reference example):
   - Overview
   - Design goals table with **requirement UIDs** in the target column
   - Scope (in / out)
   - Component sections
   - Link to the relevant generated C4 component view
4. **Use cases** describe missions and derive requirements: end each use case
   with a "Derived requirements" section listing UIDs (create them via the
   `requirements` skill).

## When code changes

Any change to nodes, topics, services, or launch remappings:

```bash
python scripts/generate_c4.py          # regenerate views
python scripts/generate_c4.py --check  # CI/drift check — must pass before merge
```

Commit regenerated views together with the code change. Check
`c4/topics.md` afterwards — new **remap warnings** or **dangling interfaces**
usually mean a launch file is missing a remapping, not that the diagram is wrong.

## Agent workforce note

Architecture docs → `infra` agent (`orchestrator` queue). When a plan changes
node interfaces, add a final `infra` step: "regenerate C4 views
(`python scripts/generate_c4.py`) and commit".
