# FOC Motor Control Library

## Overview
This repository provides a reusable Field-Oriented Control (FOC) library for single-motor embedded targets.
The library keeps control logic platform-agnostic and exposes a platform API contract for board-specific integration.

## Current Version
- v0.4.0

## Repository Layout
```text
FOC_VSCODE/
├── foc/                       # Reusable library core
│   ├── include/
│   │   ├── interface/         # API contracts and module interfaces
│   │   ├── algorithm/         # L1-L3 algorithm headers
│   │   └── config/            # Configuration and shared type headers
│   ├── src/
│   │   ├── interface/         # Interface-layer implementations
│   │   └── algorithm/         # Algorithm-layer implementations
│   └── port/                  # Empty platform API template for new ports
├── examples/                  # Concrete board/project instances
├── docs/                      # Library-only documents
└── .github/                   # Workflow and agent orchestration docs
```

## Workspace Usage
- Open `Project.code-workspace` at repository root for management, review, and documentation work.
- Open `examples/<instance>/software/Project.code-workspace` for instance build/flash/debug.

## Example Instances
- `examples/GD32F303_FOCExplore/` (reference instance)

## Documentation
- [Library Docs Index](docs/README.md)
- [Architecture](docs/architecture.md)
- [Development Guide](docs/development.md)
- [Structure and Dependency Tree](docs/structure-and-dependency-tree.md)
- [Instance Guide: GD32F303_FOCExplore](examples/GD32F303_FOCExplore/README.md)

## Contributing
- Follow rules in `docs/engineering/dev-guidelines/rules/`.
- Keep library docs and instance docs separated by ownership.
- Update documentation in the same iteration as code/config changes.

## License
MIT License. See [LICENSE](LICENSE).
