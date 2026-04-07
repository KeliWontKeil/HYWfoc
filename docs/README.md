# Documentation Index

## Scope
This folder stores library-only documents (architecture, API contracts, generic workflow, and transport-agnostic protocol definitions).

Board/project-specific documents must stay under each instance folder (for example `examples/GD32F303_FOCExplore/`).

### Boundary Rules
- Keep in `docs/`: reusable library behavior, dependency contracts, generic process rules.
- Keep in `examples/<instance>/`: hardware pin map, toolchain setup, build/debug steps, transport channel mapping.
- Library docs may link to instances, but must not contain instance-only implementation details.

## Current Baseline
- Version: v0.4.0
- Code baseline: L3 communication aggregation is implemented in `protocol_parser`, and platform communication APIs are exposed as per-source interfaces.
- Runtime baseline: fault state gates runtime sensor read chain and debug stream output; I2C timeout/recovery uses loop budgets in driver path.
- Config baseline: project configuration converges through `foc_config.h` with `foc_cfg_*` split headers.

## Documents
- `architecture.md`: Layering model, timing architecture, data flow, and module dependency constraints.
- `development.md`: Generic development workflow, coding standards, and validation requirements.
- `structure-and-dependency-tree.md`: Current file tree and layered dependency snapshot.
- `protocol-parameters-bilingual.md`: Transport-agnostic command protocol and runtime parameter reference (Chinese/English).
- `api-unused-interface-evaluation.md`: Unused API classification and keep/remove rationale.
- `library-structuring-p1.md`: P1 library-structuring migration report and file mapping.

## Related Instance Docs
- `../examples/GD32F303_FOCExplore/README.md`
- `../examples/GD32F303_FOCExplore/DEVELOPMENT.md`
- `../examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md`
- `../examples/GD32F303_FOCExplore/hardware/hardware.md`

## Recommended Reading Order
1. `architecture.md`
2. `development.md`
3. `structure-and-dependency-tree.md`
4. `protocol-parameters-bilingual.md`

## Maintenance Rules
- Whenever API signatures, config headers, or layer ownership changes, update this folder in the same iteration.
- `CHANGELOG.md` must reference the same version baseline as this index.
- `NEXT_MISSION.md` target version should always be one step ahead of the current baseline.
