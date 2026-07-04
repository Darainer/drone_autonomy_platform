---
name: test-plan
description: Create or update a test plan under docs/test_plans/ and link tests to requirement UIDs with Verifies markers. Use when planning verification for requirements or a design, when writing tests that verify requirements, or when a traceability matrix row shows a requirement as uncovered.
---

# Test Planning & Requirement Linkage

Test plans live in `docs/test_plans/TP-<NNN>-<slug>.md` (see
`TP-001-latency-and-safety.md` as the reference example). Every **Approved**
requirement in `docs/requirements/*.sdoc` must appear in exactly one test plan.

## Verification methods (DO-178C vocabulary)

| Method | Meaning | Traceability |
|---|---|---|
| **Test** | Automated test executes the behavior | `Verifies:` marker in test code |
| **Analysis** | Derived from other measurements/budgets | Test plan row only |
| **Inspection** | Datasheet/doc/code review | Test plan row only |
| **Demonstration** | Manual/field procedure | Test plan row + report in `docs/reports/` |

Safety-critical requirements (`safety-critical` tag) must use **Test** wherever
technically possible, and may not ship while still `planned`.

## Test plan template

```markdown
# TP-<NNN> — <Title>

**Status:** Draft | Active | Complete
**Requirements document:** link to the .sdoc

## Scope

## Requirements Coverage
| UID | Method | Environment | Test / Procedure | Status |
|---|---|---|---|---|
| SAF-1 | Test | SITL | <concrete procedure with pass criteria> | planned/implemented |

## Test Environments
(unit pytest / smoke_test.sh / SITL via sim-test queue / HIL / field)

## Exit Criteria
```

Every row needs a **concrete pass criterion** (threshold, count, exact
expected call) — "verify it works" is not a procedure.

## Linking tests to requirements

Put a marker comment directly above the test (comma-separate multiple UIDs):

```python
# Verifies: SAF-1
def test_critical_battery_triggers_rtl_once():
    ...
```

```cpp
// Verifies: COMP-6
TEST(ControlNode, CommandGenerationUnder10ms) { ... }
```

`scripts/check_traceability.py` picks these up from any file with `test` in
its name or a `tests/` directory. Once a marker lands, flip the test plan row
from `planned` to `implemented`.

## Test environments in this repo

- **Unit:** `pytest` over `agents/tests/` (see `pyproject.toml`); ROS2 package
  tests go next to the package.
- **Smoke:** `scripts/smoke_test.sh` inside the platform Docker image
  (checks expected nodes are up — extend its node list when adding nodes).
- **SITL:** plan a `sim-test` step on the `simulation` queue via
  `scripts/submit_task.py`.

## Every change ends with

```bash
python scripts/check_traceability.py           # refresh matrix, commit it
python scripts/check_traceability.py --strict  # must pass: every approved req has an
                                                # implemented Verifies: test (not just "planned")
```

Then check the matrix: rows you planned should show 🟡 planned, rows with
implemented tests ✅ verified.
