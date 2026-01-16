# Test Generation Agent

**Purpose:** This agent generates test cases for new features.

**Trigger:** This agent is triggered when a pull request is marked as "feature ready".

**Actions:**

1.  Analyzes the code in the pull request to understand the new feature.
2.  Generates a set of test cases that cover the new feature.
3.  Creates a new pull request with the generated test cases.
