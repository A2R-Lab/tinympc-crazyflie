# Agent Guide

### Agent delegation

For complex work, the primary agent acts as lead and orchestrator and retains responsibility for architectural consistency, correctness, integration, and final verification. Apply this policy to all project work, including goal-backed tasks such as those created through `$define-goal`.

- Reserve the primary agent's higher reasoning effort for planning, architecture, ambiguous decisions, difficult debugging, integration, and final verification.
- Proactively delegate well-defined, bounded tasks when delegation can reduce token or compute usage or parallelize independent work. Prefer lower reasoning effort and/or cheaper models for routine searches, file discovery, documentation lookup, straightforward implementation, test execution, log inspection, formatting, and similar mechanical work; use stronger reasoning for a subagent when its task genuinely requires it.
- Give each subagent a narrow task, sufficient context and constraints, and a clear expected output. Independently verify important conclusions before relying on them.
- Avoid delegation when coordination overhead would exceed the expected benefit.
