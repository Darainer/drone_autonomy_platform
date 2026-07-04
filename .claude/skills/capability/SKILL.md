---
name: capability
description: Stakeholder-level capability engineering — capture what task the platform must accomplish (STK requirements), define the target architecture, measure the gap to the current code, and hand off work packages to the detailed implementation loop. Use when asked "can/should the platform do X", for new mission-level capabilities, ConOps, gap analysis, target vs current architecture, or capability status.
---

# Capability Loop (Stakeholder → Target Architecture → Gap → Handoff)

This is the outermost of the three engineering loops:

1. **Capability loop (this skill, designer-owned):** what task must the
   platform accomplish? Does the software as-built actually produce that
   result? What architecture would? → iterated artifacts + gap report.
2. **System loop:** `requirements` / `design` / `test-plan` skills — derived
   requirements, DES docs, verification linkage.
3. **Implementation loop:** Claude Code sessions or the agent workforce
   (`scripts/submit_task.py`) executing work packages.

## Artifacts (one set per capability)

| Artifact | Location | Owner |
|---|---|---|
| Stakeholder requirement `STK-<n>` | `docs/requirements/stakeholder_requirements.sdoc` | designer |
| Capability doc `CAP-<NNN>-<slug>.md` | `docs/capabilities/` | designer |
| Target architecture spec | `docs/architecture/target/CAP-<NNN>-<slug>.yaml` | designer |
| Gap report (generated) | `docs/reports/gap_CAP-<NNN>.md` | `scripts/check_architecture_gap.py` |

Reference example: CAP-001 (photogrammetry/survey mapping).

## The designer iteration loop

1. **Capture the need** as `STK-<n>` (via the `requirements` skill; Draft
   while iterating). The STATEMENT is the task outcome ("fly a survey
   mission and deliver a complete 3D map post-flight"), the COMMENT holds
   measurable **acceptance criteria** — capabilities are *validated*
   (does it accomplish the task?), not just verified.
2. **Write the capability doc** (`docs/capabilities/CAP-<NNN>-<slug>.md`)
   with sections: Stakeholder need · ConOps (numbered operational sequence) ·
   Target architecture (Mermaid C4 container view — target, not current) ·
   Gap to current · Requirements derived · Implementation handoff ·
   Validation plan · Designer iteration log.
3. **Encode the target architecture** as YAML so the gap is machine-checked:
   - `containers`: `kind: node` (checked against parsed ROS2 nodes),
     `kind: offboard` (+ `path:`, checked for existence),
     `kind: external` (checked against `EXTERNAL_SYSTEMS`)
   - `flows`: `{from, to, topic}` checked against the current topic graph
     (remappings applied — same model as the `c4` skill)
   - `behaviors`: `{name, requirement, evidence}` where `evidence` is
     normally an `Implements: <UID>` marker — behavioral gaps inside
     existing containers flip to present when the marked code lands.
4. **Measure:** `python scripts/check_architecture_gap.py` → report shows
   present/missing per element and a gap checklist. Iterate 2–4 until the
   target is right (change the target freely at this stage — that's the
   point of the loop).
5. **Derive system requirements** for each gap cluster (`requirements`
   skill; family per subsystem, `Parent: STK-<n>`), and reference their UIDs
   from the YAML so gaps and requirements stay linked.
6. **Decompose into work packages** in the capability doc's handoff table:
   each WP = one design doc + one implementation session, with agents/queue
   and machine-checkable exit criteria ("these gap lines flip to ✅").
7. **Approve & hand off** (see below). Rerun the gap check after every
   implementation merge; the capability is **complete when the gap report
   reads N/N present** and the validation plan has passed.

## Handoff contract (to the detailed loop)

A work package handed to an implementation session (Opus/Sonnet-class Claude
Code or `submit_task.py`) must contain:

- the DES doc to write/implement (`design` skill) and the requirement UIDs,
- the target-spec lines it must satisfy (its exit = those lines ✅ under
  `python scripts/check_architecture_gap.py`),
- the traceability markers expected (`Implements:`/`Verifies:` — the gap
  checker greps for behavior evidence, so unmarked code does not count),
- `safety_critical: true` if `src/control|safety|navigation` is touched.

The implementing session must NOT edit the target YAML or the capability doc
except to append to the iteration log — target changes are designer decisions
that come back through this loop.

## Answering "does the software actually do X?"

Run the gap check, then read three generated sources together:
`docs/reports/gap_CAP-<NNN>.md` (structure/behavior),
`docs/reports/traceability_matrix.md` (requirement coverage), and
`docs/architecture/c4/topics.md` (is the data path actually connected).
A capability claim needs all three: elements present, requirements verified,
validation passed. Report format: `report` skill.
