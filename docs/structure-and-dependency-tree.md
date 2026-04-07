# Project Structure and Dependency Tree

## Purpose
This document records the repository layout after library-first reorganization and the dependency direction constraints.

## File Tree (Current Core)
```text
FOC_VSCODE/
├── docs/                              # library-only documents
├── foc/                               # reusable FOC library
│   ├── include/
│   │   ├── interface/                 # API contracts
│   │   ├── algorithm/                 # L1-L3 algorithm headers
│   │   └── config/                    # config macros + shared types
│   ├── src/
│   │   ├── interface/                 # interface-layer implementations
│   │   └── algorithm/                 # algorithm-layer implementations
│   └── port/                          # platform contract empty/stub template
└── examples/
	└── <instance>/                     # concrete board/project instance
	    ├── hardware/                   # instance hardware docs
	    └── software/                   # instance standalone project package
```

## Layered Dependency Tree
```text
L1/L2/L3 Core Library
└── foc/src/{interface,algorithm} + foc/include/{interface,algorithm,config}

Special Dependency Layer Contract
├── foc/include/interface/foc_platform_api.h
├── foc/include/config/foc_shared_types.h
└── foc/include/config/foc_cfg_*.h

Board-Specific Example Instance
├── software/Application/*.c (main/isr/platform_api)
├── software/Utilities/* (L4 drivers)
└── software/Firmware/* (vendor library)
```

## Dependency Tree (Code-Level)
```text
example main.c -> foc_app API

example project file -> foc/src canonical sources (external reference)

foc/src/interface -> foc/include/interface + foc/include/config

foc/src/algorithm -> foc/include/algorithm + foc/include/config + foc/include/interface

foc (L1/L2/L3) -> interface/foc_platform_api.h contract only

example platform API implementation -> Utilities/* (L4)

Utilities/* -> Firmware/* (vendor library)
```

## Current Compliance Snapshot
- Pass: L1/L2/L3 canonical sources are centralized in `foc/src/{interface,algorithm}`.
- Pass: example project references library sources directly (no root wrapper dependency).
- Pass: board-specific drivers and vendor firmware are bundled in example `software` for standalone build portability.
- Pass: instance-local `.pack/.cmsis/.eide/build` assets are owned by each `software` instance.
- Pass: root `docs` is scoped to library-oriented documentation.

## Review Checklist
- `foc` must not directly include board driver headers.
- example project must build with `software/Firmware` + `software/Utilities` + external `foc` reference.
- root workspace is management-only; compile/debug entry must be per-instance workspace.
- platform differences must stay in `examples/GD32F303_FOCExplore/software/Application/Source/foc_platform_api.c`.
- library documents should remain in root `docs`; hardware/board docs should remain in each example instance.
