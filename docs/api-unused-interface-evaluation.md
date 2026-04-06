# API Unused Interface Evaluation

## Scope
- Reviewed layers: L1/L2/L3 and special layer under Application.
- Excluded: Utilities (L4) and vendor firmware.

## Classification Rules
- RESERVED_INTERFACE: intentionally kept for future extension or debug hooks.
- REFACTOR_RESIDUAL: left after refactor and currently not required by runtime behavior.
- MISSING_WIRING: feature is designed and useful now, but call path is not connected.

## Evaluated Symbols

| Symbol | Location | Current Usage | Classification | Decision |
|---|---|---|---|---|
| DebugStream_Init | Application/Include/debug_stream.h, Application/Source/debug_stream.c | Was not called in init path | MISSING_WIRING | Connected in FOC_App_Init and kept as explicit reset entry |
| SVPWM_GetOutput | Application/Include/svpwm.h, Application/Source/svpwm.c | No internal call sites | RESERVED_INTERFACE | Keep with audit tag for optional observability/debug |
| ControlScheduler_ResetTickCounter | Application/Include/control_scheduler.h, Application/Source/control_scheduler.c | No internal call sites | RESERVED_INTERFACE | Keep with audit tag for deterministic test/re-sync scenarios |

## Next Release Candidates
- If SVPWM_GetOutput still has no external consumer after one release, reclassify to REFACTOR_RESIDUAL and prepare removal.
- If ControlScheduler_ResetTickCounter is still unused after test harness integration review, reclassify to REFACTOR_RESIDUAL.

## Notes
- This document records intent so future cleanup can remove dead APIs safely instead of by guesswork.
