# Project Structure and Dependency Tree

## Purpose
This document gives a quick visual view of current folder layout and layered dependency paths using the refined 4-layer + special-layer model.

## File Tree (Current Core)
```text
FOC_VSCODE/
├── Application/
│   ├── Include/
│   │   ├── main.h
│   │   ├── foc_app.h
│   │   ├── foc_config.h
│   │   ├── foc_config_*.h
│   │   ├── foc_platform_api.h
│   │   ├── control_scheduler.h
│   │   ├── foc_control.h
│   │   ├── foc_control_init.h
│   │   ├── foc_control_internal.h
│   │   ├── command_manager.h
│   │   ├── protocol_parser.h
│   │   ├── debug_stream.h
│   │   └── ...
│   └── Source/
│       ├── main.c
│       ├── foc_app.c
│       ├── foc_platform_api.c
│       ├── control_scheduler.c
│       ├── foc_control.c
│       ├── foc_control_init.c
│       ├── math_transforms.c
│       ├── sensor.c
│       ├── command_manager.c
│       ├── protocol_parser.c
│       ├── debug_stream.c
│       └── ...
├── Utilities/
│   ├── ADC/
│   ├── AS5600/
│   ├── I2C/
│   ├── LED/
│   ├── PWM/
│   ├── TIMER1/
│   ├── TIMER2/
│   └── USART/
├── Firmware/
├── docs/
│   ├── README.md
│   ├── architecture.md
│   ├── development.md
│   ├── hardware.md
│   └── structure-and-dependency-tree.md
├── dev-guidelines/
│   ├── rules/
│   └── skills/
├── build/
└── output/
```

## Layered Dependency Tree
```text
L1 Application Layer
└── main.c
    └── foc_app.c

L2 Algorithm Layer
├── foc_control.c
├── foc_control_init.c
├── control_scheduler.c
└── command_manager.c

L3 Advanced Peripheral Layer
├── sensor.c
├── svpwm.c
├── debug_stream.c
└── protocol_parser.c

Special Dependency Layer
├── foc_platform_api.c/.h
├── foc_shared_types.h
├── math_transforms.c/.h
└── config headers (implemented, domain-split)

L4 Utilities Drivers
├── adc.c / as5600.c / i2c.c
├── pwm.c / timer1.c / timer2.c
├── usart1.c / usart2.c / comm_frame_mux.c
├── led.c / systick.c
└── other utility modules
```

## Dependency Tree (Code-Level)
```text
main.c -> foc_app.c

foc_app.c -> (L2/L3 public APIs)

L2 modules -> Special layer APIs/types/macros
L3 modules -> Special layer APIs/types/macros

Special layer -> Utilities/* (L4)

Utilities/* -> GD32 std peripheral library

gd32f30x_it.c
├── adc internal IRQ handler
├── usart1 internal IRQ handler
├── usart2 internal IRQ handler
├── timer1 internal IRQ handler
├── timer2 internal IRQ handler
└── dma internal IRQ handler
```

## Current Compliance Snapshot
- Pass: `main.c` only calls app-layer entry.
- Pass: Utilities do not include app/algorithm headers.
- Risk: special-layer header still exposes L4 headers upward (`foc_platform_api.h`).
- Pass: L3 `svpwm` no longer directly includes L4 `pwm.h`; PWM access now routes through special layer API.

## Review Checklist
- L1 should only depend on foc_app public API.
- L2 should not directly include or call Utilities headers/APIs.
- L3 should not directly include Utilities headers/APIs.
- L1/L2/L3 must access L4 via Special Dependency Layer only.
- L4 should remain hardware-focused and reusable.
- `foc_control_internal.h` is an L2-only bridge header and should not be included by L1/L3/L4 modules.
