# Development Guide

## Scope
This guide defines reusable library development workflow and coding constraints.
Instance-specific build/debug setup belongs to `examples/<instance>/DEVELOPMENT.md`.

## Workspace Responsibilities
- Root workspace (`Project.code-workspace`): repository management, code review, documentation maintenance.
- Instance workspace (`examples/<instance>/software/Project.code-workspace`): build/flash/debug entry.

## Development Workflow

### 1. Planning
- Define scope in `NEXT_MISSION.md`.
- Confirm layer ownership and expected impact.

### 2. Implementation
- Follow rules in `docs/engineering/dev-guidelines/rules/`.
- Default to direct implementation on `main` unless a dedicated branch is explicitly requested.
- Keep L1-L3 logic platform-agnostic.

### 3. Verification
- Build with no newly introduced warnings.
- Validate layer boundaries and include dependencies.
- Perform instance-level runtime/hardware validation when required.

### 4. Documentation Sync
- Update library docs for architecture/API/contract changes.
- Update instance docs for hardware/toolchain/channel-mapping changes.

### 5. Release
- Review change scope and risks.
- Update `CHANGELOG.md`.
- Create tags only when explicitly requested.

## Coding Standards

### Dependency Layer Contract
- L1 application layer: externally callable app APIs.
- L2 algorithm layer: control/scheduler/command/debug runtime logic.
- L3 advanced peripheral layer: conversion and parsing logic.
- L4 peripheral layer: instance-owned board drivers only.
- Special dependency layer: platform API contract, shared structs, and config macros.

### Config Convergence Rule
- Use `config/foc_config.h` and `config/foc_cfg_*.h` as single source for configurable constants.
- Do not scatter configurable defaults or limits across `.c` files.

### Language Rules
- C99 with embedded-safe practices.
- Include `<stddef.h>` for `NULL` when required.
- Keep hard real-time paths non-blocking.

### Naming Conventions
- Functions: `ModuleName_FunctionName`
- Variables: `camelCase`
- Macros: `UPPER_CASE`
- Types: `type_name_t`
- Structs: `Module_Struct`

## Quality Gates
- No newly introduced warnings in production build.
- Documentation updated in the same iteration.
- Interface contract changes reviewed against instance implementation impact.

## Troubleshooting (Library-Side)
- Build errors: verify include path and moved-file references.
- Runtime mismatch: verify instance platform API implementation against interface contract.
- Protocol mismatch: verify instance channel adaptation doc and runtime transport wiring.

## Instance References
- `../examples/GD32F303_FOCExplore/DEVELOPMENT.md`
- `../examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md`
