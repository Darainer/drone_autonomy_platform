# Safety Review Agent

> **Status: inert.** No workflow in `.github/workflows/` implements this file's
> trigger. This document describes an intended agent that does not run today;
> the only thing that actually fires on safety-critical path changes is the
> advisory `safety-path-warning` CI job in `.github/workflows/ci.yml` (see
> `CONTRIBUTING.md`). Treat everything below as planned, not live.

**Purpose:** This agent analyzes safety-critical code to identify potential issues.

**Trigger:** This agent is triggered when a pull request is opened that modifies code in the `src/control`, `src/safety`, or `src/navigation` directories.

**Actions:**

1.  Performs a static analysis of the code to identify potential safety violations.
2.  Runs a set of safety-related tests.
3.  Posts a comment on the pull request with a summary of its findings.
4.  If any potential safety violations are found, the agent will request changes on the pull request.
