# Documentation Index

## Scope
This folder stores FOC library documents only (architecture, API/contract, development workflow, and protocol references).

Example-project documents are stored under each example instance, e.g. `examples/GD32F303_FOCExplore/hardware/` and `examples/GD32F303_FOCExplore/software/`.

## Current Baseline
- Version: v0.4.0
- Code baseline: L3 communication aggregation is implemented in `protocol_parser`, and platform communication APIs are exposed as per-source interfaces.
- Runtime baseline: fault state gates runtime sensor read chain and debug stream output; I2C timeout/recovery uses loop budgets in driver path.
- Config baseline: project configuration converges through `foc_config.h` with `foc_cfg_*` split headers.

## Documents
- `architecture.md`: Layering model, timing architecture, data flow, and module dependency constraints.
- `development.md`: Build/debug workflow, coding standards, and validation requirements.
- `structure-and-dependency-tree.md`: Current file tree and layered dependency snapshot.
- `protocol-parameters-bilingual.md`: Command protocol and runtime parameter reference (Chinese/English).
- `api-unused-interface-evaluation.md`: Unused API classification and keep/remove rationale.
- `library-structuring-p1.md`: P1 library-structuring migration report and file mapping.

## Recommended Reading Order
1. `architecture.md`
2. `development.md`
3. `structure-and-dependency-tree.md`
4. `protocol-parameters-bilingual.md`

## Maintenance Rules
- Whenever API signatures, config headers, or layer ownership changes, update this folder in the same iteration.
- `CHANGELOG.md` must reference the same version baseline as this index.
- `NEXT_MISSION.md` target version should always be one step ahead of the current baseline.
