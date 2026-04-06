# P1 Library Structuring Report

## Scope
- Goal: complete P1 library structuring for single-motor FOC open-source direction.
- Current state: migration has moved from transitional layout to instance-oriented layout.

## Status Note
- This document was initially created in transitional phase (`foc-lib` + wrapper strategy).
- It is now updated to reflect the current settled structure (`foc` + standalone example instance).

## Current Top-Level Structure (P1 settled)
- `foc/`: reusable L1-L3 + config + API contract.
- `examples/GD32F303_FOCExplore/software/`: standalone GD32 software project instance.
- `examples/GD32F303_FOCExplore/hardware/`: board-level hardware documents.
- `docs/`: library-only documents.

## Ownership Boundaries
- Core reusable library:
  - `foc/include/*.h`
  - `foc/src/*.c`
  - `foc/port/foc_platform_api_empty.c`
- Example-specific implementation:
  - `examples/GD32F303_FOCExplore/software/Application/Source/foc_platform_api.c`
  - `examples/GD32F303_FOCExplore/software/Utilities/*`
  - `examples/GD32F303_FOCExplore/software/Firmware/*`

## API Split
- Empty API template outside example:
  - `foc/port/foc_platform_api_empty.c`
- Implemented API inside example:
  - `examples/GD32F303_FOCExplore/software/Application/Source/foc_platform_api.c`

## Migration Result Summary
- `foc-lib` renamed to `foc`.
- Root-level `Firmware` and `Utilities` migrated into `examples/GD32F303_FOCExplore/software`.
- uVision project under example software now references core sources from external `../../../foc/src`.
- Hardware document moved from root docs to `examples/GD32F303_FOCExplore/hardware/hardware.md`.

## Compatibility Strategy Update
- Transitional wrapper layer has been retired from active build flow.
- Legacy root-level assets are archived under:
  - `examples/GD32F303_FOCExplore/software/_legacy_from_root/`

## Notes
- This phase focused on restructuring and ownership clarification; algorithm/protocol behavior was not intentionally changed.
- Follow-up phases should continue with open-source governance files and documentation SSOT cleanup.