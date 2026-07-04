---
name: design
description: Write or update a feature/change design document under docs/design/ before implementation. Use when planning a new node, algorithm, interface, or refactor; when the user asks for a design, RFC, or proposal; or before submitting a multi-step implementation plan to the agent workforce.
---

# Design Document Workflow

Design docs live in `docs/design/DES-<NNN>-<slug>.md` (`NNN` sequential —
check existing files for the next number). A design is required before any
change that adds a node, changes a topic/service/message interface, or touches
a safety-critical package.

## Process

1. **Requirements first.** Every design must list the requirement UIDs it
   addresses. If none exist yet, invoke the `requirements` skill and create
   them (Draft is fine) before writing the design.
2. **Ground it in current architecture.** Read `docs/architecture/c4/`
   (regenerate with `python scripts/generate_c4.py` if node/topic code changed
   recently) and link the relevant views instead of redrawing them.
3. Write the doc from the template below.
4. **Interfaces are contracts.** New message types go in `msgs/ros2/` and need
   an `infra` plan step before any node work; new topics must appear in the
   Interfaces table with full name, type, and QoS.
5. Finish with a test strategy — then invoke the `test-plan` skill to turn it
   into a tracked plan.

## Template

```markdown
# DES-<NNN> — <Title>

**Status:** Draft | Approved | Implemented | Superseded
**Safety-critical:** yes/no (yes if src/control, src/safety, or src/navigation is touched)

## Summary
One paragraph: what changes and why.

## Requirements addressed
| UID | How this design satisfies it |
|---|---|
| COMP-3 | ... |

## Current state
Link to docs/architecture/c4/ views and relevant architecture docs.

## Proposed design
Structure, node responsibilities, data flow. Diagrams in Mermaid.

## Interfaces
| Direction | Topic/Service | Type | QoS | Notes |
|---|---|---|---|---|

## Safety impact
Failure modes, failsafe interaction, DO-178C considerations (or "none").

## Test strategy
How each requirement above will be verified (method + environment).
Link the test plan (docs/test_plans/TP-*.md) once it exists.

## Alternatives considered
What was rejected and why.

## Open questions
```

## Rules

- Status transitions: `Draft` → `Approved` (human sign-off) → `Implemented`
  (after code merges — also regenerate C4 views, see the `c4` skill) →
  `Superseded` (link the successor).
- A design that changes topics/nodes is not `Implemented` until
  `python scripts/generate_c4.py --check` passes on the merged code.
- Safety-critical designs require the human approval gate noted in
  CONTRIBUTING.md; mark the corresponding `submit_task.py` plan
  `safety_critical: true`.

## Agent workforce note

Design docs themselves → `infra` agent (`orchestrator` queue). The design's
"Proposed design" section should map cleanly onto plan steps for the domain
agents (`perception-dev`, `nav-dev`, …) — write it with that decomposition in
mind.
