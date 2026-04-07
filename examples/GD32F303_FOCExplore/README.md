# GD32F303_FOCExplore Example

This folder is a complete board instance for GD32F303-based FOC bring-up.

## Hardware Requirements
- GD32F303CC microcontroller
- ST-LINK debugger
- Current sensing front-end
- AS5600 magnetic encoder
- Compatible motor driver stage

## Development Environment
- Keil uVision 5 (ARM Compiler 5)
- VS Code with EIDE extension
- ST-LINK tooling

## Layout
- `hardware/`: board-level documentation, wiring, and pin map.
- `software/`: standalone project package for build/flash/debug.
- `DEVELOPMENT.md`: instance-specific build/debug configuration and workflow.
- `PROTOCOL_ADAPTATION.md`: instance communication channel mapping.

## Quick Start
1. Open `software/Project.uvprojx` in Keil, or open `software/Project.code-workspace` in VS Code.
2. Build and flash from instance workspace only.
3. Validate runtime behavior using the channel mapping in `PROTOCOL_ADAPTATION.md`.

## Design Rules
- This instance keeps board-specific drivers and vendor firmware locally under `software/`.
- Core control library is referenced from external `foc/` (single source of truth).
- Any board-specific platform API adaptation must stay inside `software/Application/Source/foc_platform_api.c`.
