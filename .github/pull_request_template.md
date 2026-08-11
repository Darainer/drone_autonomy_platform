<!--
Fill in every section. Delete this comment block before submitting.
See docs/standards/do_178c_context.md for the DO-178C context this checklist
is drawn from.
-->

## Summary

<!-- What does this PR do, and why? -->

## Safety-critical path declaration

Does this PR touch any of the platform's designated safety-critical paths —
`src/control/`, `src/safety/`, or `src/navigation/`?

- [ ] Yes, this PR changes files under one or more of those paths.
- [ ] No, this PR does not touch any safety-critical path.

If **yes**, complete the DO-178C section below and add the `safety-reviewed`
label once a human has reviewed the change. A CI job posts an advisory
warning to this PR's job summary when a safety-critical path changes without
that label — it is advisory only and does not block merge on its own.

## DO-178C checklist (required if the box above is checked "Yes")

DO-178C is objective-based, not a fixed procedure — use this checklist to
show *how* the relevant objectives are satisfied for this change, not just
that a box was ticked.

- [ ] **DAL assigned.** State the Design Assurance Level (A–E) that applies to
      the changed component(s) and briefly justify it (what failure condition
      drives the level — catastrophic, hazardous, major, minor, no safety
      effect).

      DAL: `___`  Justification: <!-- one or two sentences -->

- [ ] **Objective-satisfaction rationale.** Describe which DO-178C objectives
      this change bears on (e.g. requirements-based testing, traceability,
      verification of derived requirements) and how this PR satisfies them —
      not a citation, an explanation of what was actually done.

- [ ] **Verification evidence proportional to DAL.** Attach or link the
      evidence appropriate to the assigned DAL (test results, review notes,
      analysis). Higher DALs need more rigorous evidence; say what level of
      evidence is included here and why it's sufficient.

- [ ] **Traceability markers present.** Every changed requirement-bearing line
      carries `Implements:`/`Verifies:` markers linking to requirement UIDs in
      `docs/requirements/*.sdoc`. List the UIDs touched:

      <!-- e.g. Implements: SAF-1, Verifies: SAF-1 -->

## C4 / architecture drift

- [ ] `python scripts/generate_c4.py --check --no-render` exits 0 (no drift),
      or the regenerated views are included in this PR.

## Traceability / checker gates

- [ ] `python scripts/check_traceability.py --strict` exits 0, or the
      traceability matrix update is included in this PR.
- [ ] `python scripts/check_architecture_gap.py --strict` exits 0 (only
      relevant if this PR merges a capability work package or edits a target
      architecture spec).

## Reviewer notes

<!-- Anything a reviewer should look at specifically, known limitations,
     follow-up work intentionally left out of scope. -->
