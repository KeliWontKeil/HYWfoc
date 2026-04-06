# GD32F303_FOCExplore Software Instance

This folder is intended to be opened directly by IDE tools.

## Contents
- `Project.uvprojx`, `Project.uvoptx`: uVision project files.
- `Application/`: board entry (`main.c`), ISR bridge, and platform API implementation.
- `Utilities/`: board-specific driver modules.
- `Firmware/`: bundled GD32 vendor firmware for independent build.
- `_legacy_from_root/`: migration archive from root-level legacy layout.

## Build Dependency
- Core library sources are referenced from external path: `../../../foc/src`.
- Core headers are referenced from external path: `../../../foc/include`.

## Notes
- Keep this instance self-contained for board support files.
- Do not copy `foc` sources into this folder; keep `foc` as canonical library.
