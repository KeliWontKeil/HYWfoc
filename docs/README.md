# Documentation Index

## Scope
This folder contains architecture, hardware, development workflow, and protocol references for the GD32F303CC FOC project.

## Current Baseline
- Version: v0.3.8
- Code baseline: L3 communication aggregation is implemented in `protocol_parser`, and platform communication APIs are exposed as per-source interfaces.
- Config baseline: project configuration converges through `foc_config.h` with `foc_cfg_*` split headers.

## Documents
- `architecture.md`: Layering model, timing architecture, data flow, and module dependency constraints.
- `hardware.md`: Board-level pin mapping, peripheral wiring, and electrical constraints.
- `development.md`: Build/debug workflow, coding standards, and validation requirements.
- `structure-and-dependency-tree.md`: Current file tree and layered dependency snapshot.
- `protocol-parameters-bilingual.md`: Command protocol and runtime parameter reference (Chinese/English).

## Recommended Reading Order
1. `architecture.md`
2. `hardware.md`
3. `development.md`
4. `structure-and-dependency-tree.md`
5. `protocol-parameters-bilingual.md`

## Maintenance Rules
- Whenever API signatures, config headers, or layer ownership changes, update this folder in the same iteration.
- `CHANGELOG.md` must reference the same version baseline as this index.
- `NEXT_MISSION.md` target version should always be one step ahead of the current baseline.
