# Project Structure and Dependency Tree

## Purpose
This document records the repository layout after library-first reorganization and the dependency direction constraints.

## File Tree (Current Core)
```text
FOC_VSCODE/
├── docs/                              # library-only documents
├── foc/                               # reusable FOC library
│   ├── include/                       # public headers + config headers
│   ├── src/                           # L1/L2/L3 canonical sources
│   └── port/                          # platform contract empty/stub template
└── examples/
	├── GD32F303_FOCExplore/
	│   ├── hardware/
	│   │   ├── README.md
	│   │   └── hardware.md
	│   └── software/                  # standalone project instance
	│       ├── Application/           # board entry + ISR + platform API impl
	│       ├── Utilities/             # driver layer for this board instance
	│       ├── Firmware/              # vendor firmware bundled in instance
	│       ├── Project.uvprojx
	│       ├── Project.uvoptx
	│       └── _legacy_from_root/     # temporary migration archive
	└── uvision_EIDE/                  # historical reference example (to be retired)
```

## Layered Dependency Tree
```text
L1/L2/L3 Core Library
└── foc/src/*.c + foc/include/*.h

Special Dependency Layer Contract
├── foc/include/foc_platform_api.h
├── foc/include/foc_shared_types.h
└── foc/include/foc_cfg_*.h

Board-Specific Example Instance (GD32F303_FOCExplore)
├── software/Application/*.c (main/isr/platform_api)
├── software/Utilities/* (L4 drivers)
└── software/Firmware/* (vendor library)
```

## Dependency Tree (Code-Level)
```text
example main.c -> foc_app API

example project file -> foc/src canonical sources (external reference)

foc/src -> foc/include

foc (L1/L2/L3) -> foc_platform_api.h contract only

example platform API implementation -> Utilities/* (L4)

Utilities/* -> Firmware/* (vendor library)
```

## Current Compliance Snapshot
- Pass: L1/L2/L3 canonical sources are centralized in `foc`.
- Pass: example project references library sources directly (no root wrapper dependency).
- Pass: board-specific drivers and vendor firmware are bundled in example `software` for standalone build portability.
- Pass: root `docs` is scoped to library-oriented documentation.

## Review Checklist
- `foc` must not directly include board driver headers.
- example project must build with `software/Firmware` + `software/Utilities` + external `foc` reference.
- platform differences must stay in example `Application/Source/foc_platform_api.c`.
- library documents should remain in root `docs`; hardware/board docs should remain in each example instance.
