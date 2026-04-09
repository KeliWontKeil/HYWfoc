# AI Project Initialization Template (Repository-Level)

## Context for AI Assistant

You are working in the HYWfoc repository (何易位FOC), which is organized as:

- `foc/`: reusable library core (platform-agnostic)
- `examples/`: board and toolchain specific instance projects
- `docs/`: library-level architecture and workflow documents

This initialization file is intentionally **repository-level**.

Do not assume one fixed board instance. For board-specific details, always read the corresponding instance documents under `examples/<instance>/`.

## Scope Boundary (Important)

Use this file for:
- global development workflow
- repository-level coding and collaboration rules
- versioning and commit governance

Do not place instance-specific details here, including:
- exact pin mappings
- board wiring
- instance build output paths
- transport channel bindings for one concrete board

Those details must remain in each instance doc set.

## Required Rule Sources

Before implementation, read:
- `docs/engineering/dev-guidelines/rules/cn/`
- `docs/engineering/dev-guidelines/rules/en/`
- `docs/development.md`
- `NEXT_MISSION.md`

## Global Workflow for AI-Assisted Development

1. Read mission scope and relevant rules.
2. Implement on `main` unless user explicitly requests another branch.
3. Build and validate in the target instance workspace.
4. Synchronize documentation in the same iteration.
5. Commit locally according to the commit/version policy below.

## Commit and Version Governance (Mandatory)

### Commit Rule
- After each completed modification cycle, perform **local `git commit` only**.
- **Do not `git push`** unless user explicitly requests push.

### Version Rule
- Use `1.2.3` style version semantics:
	- `1`: major version
	- `2`: pushable minor version
	- `3`: local revision number
- After each local `git commit`, local revision (`.3`) must increment by `+1`.
- Numeric values above are examples only; concrete numbers follow real project progress.

### Commit Amendment Rule
- If user requests modification to the current commit, use:
	- `git commit --amend`
- Do not create a separate new commit for that correction unless user explicitly requests it.

### Activation Point
- Governance activation baseline is **`1.0.0`** (first push milestone).
- This policy is active for all subsequent development cycles.

## Generic Technical Expectations

1. **Code Style**: Follow repository naming rules and macro conventions.
2. **Safety**: Include `<stddef.h>` when using `NULL`; avoid blocking operations in ISRs.
3. **Resources**: Keep ROM/RAM usage and real-time path cost under control.
4. **Validation**: Hardware validation remains mandatory for embedded behavior closure.

## Documentation Entry Points

- `README.md`: external-facing project introduction
- `docs/README.md`: library-level document index
- `docs/architecture.md`: layering and dependency design
- `docs/development.md`: reusable development process
- `examples/<instance>/README.md`: instance usage entry

Remember: this is an embedded control repository. Reliability, timing determinism, and hardware-verified behavior take priority over purely simulated correctness.