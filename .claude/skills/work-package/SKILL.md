---
name: work-package
description: Drive a work package through implement-then-approve — a fresh implementer subagent writes the change, a fresh reviewer subagent gates it against the WP's exit criteria, and only an APPROVE verdict commits. Use when executing a WP from docs/workflow/README.md.
disable-model-invocation: true
---

# Work package driver

Runs one work package end to end: `/work-package <WP-ID>`.

The point of the loop is that the implementer and the reviewer never share
context. A reviewer that watched the implementation rationalizes it; a reviewer
handed only the spec and the diff checks it.

## Loop

1. **Locate the spec.** Find `<WP-ID>` in `docs/workflow/README.md`. If it is not
   there, stop and ask — do not infer a spec from the ID.

2. **Implement.** Spawn a fresh `wp-implementer` subagent. Give it the WP's tasks
   and exit criteria verbatim, and nothing about previous rounds except the
   reviewer findings it must address. It edits the working tree and does not
   commit.

3. **Review.** Spawn a fresh `design-conformance` subagent. Give it the WP's
   tasks and exit criteria verbatim plus the diff. Give it no history of the
   implementation. It returns findings and a `VERDICT:` line.

4. **Act on the verdict.**
   - `APPROVE` → commit the working tree with a message naming the WP ID, then stop.
   - `REJECT` → return to step 2 with the findings, as a new implementer.

5. **Cap at 3 rounds.** After a third `REJECT`, stop and escalate to the user with
   the outstanding findings. A reviewer that manufactures findings cannot spin
   forever, and a WP that genuinely cannot converge needs a human, not a fourth try.

## Judgement the driver keeps

- A finding that blocks no exit criterion and affects no correctness is optional.
  Declining one is allowed; say which and why in the report.
- If the reviewer's finding is factually wrong about the code, verify it yourself
  before sending the implementer after it. Reviewers are not automatically right.
- If an exit criterion turns out to be impossible as written, stop and say so
  rather than weakening it silently.

## Commit convention

One commit per WP, message beginning with the WP ID:

```
WP-W3: enumerate missing elements in check_architecture_gap.py --strict output
```

Commits stack on the working branch; push once at the end of the session, not
per WP.
