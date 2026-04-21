# GD32F303_FOCExplore Software Instance

This folder is intended to be opened directly by IDE tools.

## Contents
- `Project.uvprojx`, `Project.uvoptx`: uVision project files.
- `Application/`: board entry (`main.c`), ISR bridge, and platform API implementation.
- `Utilities/`: board-specific driver modules.
- `Firmware/`: bundled GD32 vendor firmware for independent build.
- `.eide/`, `.pack/`, `.cmsis/`, `build/`: instance-local build metadata and outputs.

## Build Dependency
- Core library sources are referenced from external path: `../../../foc/src` (L1/L2/L3/L41).
- Core headers are referenced from external path: `../../../foc/include` (LS/L1/L2/L3/L41/L42).

## Notes
- Keep this instance self-contained for board support files.
- Do not copy `foc` sources into this folder; keep `foc` as canonical library.
