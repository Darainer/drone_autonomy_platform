---
name: report
description: Generate project status, traceability, and verification reports under docs/reports/. Use when asked for a status report, requirements coverage, traceability matrix, verification evidence, milestone summary, or architecture drift check.
---

# Reporting Workflow

All reports live in `docs/reports/`.

| Report | How |
|---|---|
| Traceability matrix | `python scripts/check_traceability.py` → `docs/reports/traceability_matrix.md` (generated — never hand-edit) |
| Architecture drift | `python scripts/generate_c4.py --check` (exit 1 = views stale) |
| Status / milestone / field-test reports | Hand-written: `docs/reports/YYYY-MM-DD-<topic>.md` |

## Status report procedure

1. Refresh generated artifacts first — a report over stale data is wrong:
   ```bash
   python scripts/check_traceability.py
   python scripts/generate_c4.py --check   # regenerate + commit if stale
   ```
2. Gather facts, don't guess: matrix summary counts, `git log --oneline` since
   the last report, open items from `issues/`, test results
   (`pytest agents/tests`, smoke test if runnable).
3. Write `docs/reports/YYYY-MM-DD-<topic>.md`:

```markdown
# Status Report — <topic> (<date>)

## Summary
Three sentences max: state, biggest risk, next milestone.

## Requirements & verification
Copy the Summary block from traceability_matrix.md; call out every
safety-critical (⚠) row not yet ✅ verified.

## Architecture
C4 drift check result; new/changed nodes and topics; open remap warnings and
dangling interfaces from docs/architecture/c4/topics.md.

## Work completed
Merged changes since last report, each linked to design doc / requirement UIDs.

## Risks & gaps
Uncovered requirements, failing tests, drifted docs.

## Next
Concrete, owner-assignable items.
```

## Rules

- Numbers come from the generated matrix, never from memory.
- Safety-critical (⚠) requirements without ✅ verified coverage are **always**
  listed in Risks — they cannot be omitted for brevity.
- Field-test / demonstration evidence (per the `test-plan` skill) gets its own
  dated report and is linked from the test plan row it verifies.
