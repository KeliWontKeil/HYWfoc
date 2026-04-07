# GD32F303_FOCExplore Development Guide

## Scope
This document is instance-specific and describes build/debug settings for GD32F303_FOCExplore.

## Workspace Entry
- Root workspace (`../../Project.code-workspace`): repository management only.
- Instance workspace (`software/Project.code-workspace`): build/flash/debug entry.

## Project Configuration
- Target: `GD32F30X_CL`
- Compiler: ARM Compiler 5 (AC5)
- Debug probe: ST-LINK (SWD)
- Control tick source: TIMER1 update interrupt at 1kHz
- High-rate clock source: TIMER2 at 24kHz
- PWM base: TIMER0 center-aligned output
- ADC trigger: TIMER3 compare event

## Build Outputs
- Instance-local output path: `software/build/GD32F30X_CL/`
- Expected artifacts: `Project.axf`, `Project.hex`

## Build Workflow
1. Open `software/Project.code-workspace`.
2. Run `build` task or `eide.project.build`.
3. Confirm no newly introduced warnings.
4. Flash with `build and flash` task or `eide.project.buildAndFlash`.

## Debug Workflow
- Connect ST-LINK and target board.
- Verify runtime heartbeat and communication path.
- Use protocol commands defined in `PROTOCOL_ADAPTATION.md` and `../../docs/protocol-parameters-bilingual.md`.

## Notes
- This instance owns `.eide/`, `.pack/`, `.cmsis/`, and `build/` assets.
- Do not move instance-specific settings into root `docs/`.
