# Architecture Overview

## System Design

This library implements a layered single-motor FOC architecture designed for portability across embedded platforms.
Hardware-specific behavior is isolated behind platform API contracts so L1-L3 logic can stay reusable.

## Software Layers

### Level 1: Application Layer
- `foc_app`: application-facing entry sequence and runtime orchestration.

### Level 2: Algorithm Layer
- `foc_control`: runtime control logic.
- `foc_control_init`: startup calibration and initialization logic.
- `control_scheduler`: task scheduling and rate dispatch.
- `command_manager`: command execution and runtime parameter management.
- `debug_stream`: debug/telemetry policy and formatting.

### Level 3: Advanced Peripheral Layer
- `sensor`: raw acquisition to structured sensor values.
- `svpwm`: voltage/vector command to PWM duty request.
- `protocol_parser`: frame parsing and command extraction.

### Level 4: Peripheral Layer (Instance-Owned)
- Board drivers and vendor firmware are instance-specific and live under `examples/<instance>/software/`.

### Special Dependency Layer
- `interface/foc_platform_api.h`: unified hardware abstraction contract.
- `config/foc_shared_types.h`: shared public data types.
- `config/foc_config.h` + `config/foc_cfg_*.h`: compile-time configuration source.

## Dependency Contract
- L1/L2/L3 can access L4 only through the Special Dependency Layer.
- L4 must not depend on L1/L2/L3.
- Public headers should not expose board driver headers.

## Timing Architecture (Abstract)

```text
Control Tick Source
├── periodic scheduler tick callback
├── control-loop slot
└── lower-rate service/monitor slots

High-Rate Clock Source
├── optional interpolation callback
└── high-rate actuation update support

Sampling Trigger Source
└── aligned current/angle acquisition trigger
```

Instance-specific timer/peripheral assignments belong to instance docs, not library docs.

## Data Flow

```text
Raw Acquisition -> Sensor Conversion -> FOC Control -> Actuation Request
         ^                                         |
         |-----------------------------------------|
```

## Module Dependencies

```text
foc_app
└── interface and algorithm modules

algorithm modules
└── special dependency layer only

platform API implementation (instance side)
└── board drivers (L4)
```

## Porting Notes
- Porting starts from `foc/port/foc_platform_api_empty.c`.
- Implement all required platform hooks in instance project code.
- Keep function signatures unchanged to preserve library compatibility.

## Instance References
- Example hardware mapping and wiring: `../examples/GD32F303_FOCExplore/hardware/hardware.md`
- Example project integration: `../examples/GD32F303_FOCExplore/README.md`
