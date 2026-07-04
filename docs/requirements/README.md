# Requirements

Canonical requirements for the platform live here in [StrictDoc](https://strictdoc.readthedocs.io/)
`.sdoc` format. Narrative docs (architecture, design) **reference** UIDs defined
here; they never define requirements themselves.

Workflow, UID scheme, and authoring rules: see the `requirements` skill
(`.claude/skills/requirements/SKILL.md`).

## Files

| File | Scope |
|---|---|
| `stakeholder_requirements.sdoc` | Stakeholder/mission-task requirements (STK) — owned by the `capability` skill |
| `platform_requirements.sdoc` | System requirements: latency (E2E/COMP/PLAT), functional (PER/AUT/MAP), safety (SAF) |

## Quick reference

- **UID scheme**: `<FAMILY>-<n>` — `E2E` (end-to-end), `COMP` (component budget),
  `PLAT` (platform processing), `SAF` (safety), `PER`/`NAV`/`CTL`/`AUT`/`COM`
  (package-level functional). Numbers are never reused.
- **Linkage markers** (parsed by `scripts/check_traceability.py`):
  - `// Implements: SAF-1` in source code
  - `# Verifies: PLAT-1` in test code
  - bare UID mentions in `docs/design/`, `docs/test_plans/`, `docs/architecture/`
- **Traceability matrix**: `python scripts/check_traceability.py` →
  `docs/reports/traceability_matrix.md` (`--strict` fails on any Approved
  requirement without an implemented `Verifies:` test — a test-plan mention
  alone ("planned") does not satisfy strict mode).

## Validating with StrictDoc (optional)

```bash
pip install strictdoc
strictdoc export docs/requirements --output-dir /tmp/sdoc-html   # validates + renders HTML
```

The traceability script uses its own lightweight parser and does not require
strictdoc to be installed.
