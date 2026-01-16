# Safety Review Agent

**Purpose:** This agent analyzes safety-critical code to identify potential issues.

**Trigger:** This agent is triggered when a pull request is opened that modifies code in the `src/control` or `src/safety` directories.

**Actions:**

1.  Performs a static analysis of the code to identify potential safety violations.
2.  Runs a set of safety-related tests.
3.  Posts a comment on the pull request with a summary of its findings.
4.  If any potential safety violations are found, the agent will request changes on the pull request.
