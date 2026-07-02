---
name: requirements
description: Author, change, or decompose platform requirements in StrictDoc (.sdoc) format under docs/requirements/. Use when adding a requirement, assigning a requirement UID, changing a latency/safety/functional requirement, reviewing requirement quality, or migrating requirements out of narrative docs.
---

# Requirements Workflow (StrictDoc)

Canonical requirements live in `docs/requirements/*.sdoc` (StrictDoc format).
Narrative docs (`docs/architecture/`, `docs/design/`) reference UIDs; they
never define requirements. If you find a requirement stated only in a
narrative doc, migrate it here and leave a pointer.

## UID scheme

`<FAMILY>-<n>`, sequential per family, **never reuse or renumber**:

| Family | Scope |
|---|---|
| `E2E` | End-to-end (sensor→action/display) |
| `COMP` | Component-level budgets |
| `PLAT` | Onboard platform processing |
| `SAF` | Safety / failsafe behavior |
| `PER`, `NAV`, `CTL`, `AUT`, `COM` | Package-level functional (perception, navigation, control, autonomy, communication) |

To pick the next number: `grep -h "^UID:" docs/requirements/*.sdoc | sort`.

## Authoring rules

1. **One shall per requirement.** Statement is a single, testable "shall"
   sentence with a measurable threshold. No "and/or" chains — decompose instead.
2. **Field order** (StrictDoc default grammar — order matters):
   `UID, STATUS, TAGS, TITLE, STATEMENT, RATIONALE, COMMENT`, then `RELATIONS` last.
3. **STATUS**: `Draft` → `Approved` → `Deprecated`. Never delete an Approved
   requirement; set `Deprecated` and note the successor UID in COMMENT.
4. **TAGS**: add `safety-critical` for anything affecting `src/control/`,
   `src/safety/`, or `src/navigation/` (DO-178C review scope, see
   `docs/standards/do_178c_context.md`). Also tag the domain
   (`latency`, `perception`, `failsafe`, …).
5. **RELATIONS**: derived/child requirements point at their parent:
   ```
   RELATIONS:
   - TYPE: Parent
     VALUE: E2E-2
   ```
6. Multiline fields use `STATEMENT: >>>` … `<<<`.
7. **Modern StrictDoc syntax** (validated against strictdoc ≥ 0.25): sections
   are `[[SECTION]]` … `[[/SECTION]]` (double brackets), narrative prose is a
   `[TEXT]` element with a `STATEMENT: >>>` block. The old `[SECTION]` and
   `[FREETEXT]` forms are rejected by current StrictDoc.

## Template

```
[REQUIREMENT]
UID: SAF-3
STATUS: Draft
TAGS: safety-critical, failsafe
TITLE: Short Imperative Title
STATEMENT: >>>
The platform shall <observable behavior> within <measurable bound>.
<<<
RATIONALE: >>>
Why this exists; what breaks without it.
<<<
```

## Every change ends with

1. Validate: `python scripts/check_traceability.py` must parse the file
   without errors (it also warns about markers referencing unknown UIDs).
   If `strictdoc` is installed, additionally run
   `strictdoc export docs/requirements --output-dir /tmp/sdoc-html`.
2. Regenerate the matrix (same command writes
   `docs/reports/traceability_matrix.md`) and commit it with the `.sdoc` change.
3. New/changed **Approved** requirements need verification planning — invoke
   the `test-plan` skill (a matrix row stuck at ❌ uncovered is a defect).

## Linkage markers (consumed by scripts/check_traceability.py)

- Source code implementing a requirement: `// Implements: SAF-1` (C++) or
  `# Implements: SAF-1` (Python), placed at the implementing construct.
- Tests: `# Verifies: SAF-1` — see the `test-plan` skill.
- Design docs / test plans: mention the bare UID anywhere in the text.

## Agent workforce note

When orchestrating via `scripts/submit_task.py`, requirement file edits are
docs work → route to the `infra` agent on the `orchestrator` queue. A plan
step touching `safety-critical`-tagged requirements sets `safety_critical: true`.
