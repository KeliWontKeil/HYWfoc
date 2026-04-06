# GD32F303_FOCExplore Example

This folder is a complete board instance for GD32F303-based FOC bring-up.

## Layout
- `hardware/`: board-level documentation, wiring, and pin map.
- `software/`: standalone project package for build/flash/debug.

## Design Rules
- This instance keeps board-specific drivers and vendor firmware locally under `software/`.
- Core control library is referenced from external `foc/` (single source of truth).
- Any board-specific platform API adaptation must stay inside `software/Application/Source/foc_platform_api.c`.
