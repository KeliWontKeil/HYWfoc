# Project Structure and Dependency Tree

## Purpose
This document gives a quick visual view of current folder layout and layered dependency paths after v0.3.3 development updates.

## File Tree (Current Core)
```text
FOC_VSCODE/
├── Application/
│   ├── Include/
│   │   ├── main.h
│   │   ├── foc_app.h
│   │   ├── foc_platform_api.h
│   │   ├── control_scheduler.h
│   │   ├── foc_control.h
│   │   ├── uart_debug.h
│   │   └── ...
│   └── Source/
│       ├── main.c
│       ├── foc_app.c
│       ├── foc_platform_api.c
│       ├── control_scheduler.c
│       ├── foc_control.c
│       ├── sensor.c
│       ├── uart_debug.c
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
L1 Application Entry
└── main.c
    └── foc_app.c

L2 Control Logic
├── foc_control.c
├── control_scheduler.c
└── sensor.c

L3 Platform Abstraction
├── foc_platform_api.c

L4 Utilities Drivers
├── adc.c / as5600.c / i2c.c
├── pwm.c / timer1.c / timer2.c
├── usart1.c / led.c / systick.c
└── other utility modules
```

## Dependency Tree (Code-Level)
```text
main.c
└── foc_app.c
    ├── foc_control.c
    │   └── foc_platform_api.c (mechanical angle read / wait bridge)
    ├── control_scheduler.c
    │   └── foc_platform_api.c (cycle-counter API bridge)
    ├── sensor.c
    │   └── foc_platform_api.c (ADC/encoder read wrappers)
    ├── svpwm.c
    ├── uart_debug.c
    └── foc_platform_api.c
        ├── timer1.c
        ├── timer2.c
        ├── timer3.c
        ├── adc.c
        ├── as5600.c
        ├── usart1.c
        ├── usart2.c
        ├── led.c
        └── systick.c

gd32f30x_it.c
├── adc internal IRQ handler
├── usart1 internal IRQ handler
├── usart2 internal IRQ handler
├── timer1 internal IRQ handler
├── timer2 internal IRQ handler
└── dma internal IRQ handler
```

## Review Checklist
- L1 should only depend on foc_app public API.
- L2 should not directly call L4 sensor-peripheral drivers; use L3 wrappers.
- L3 should centralize platform binding and L4 sensor-device read APIs.
- L4 should remain hardware-focused and reusable.
- Public headers should expose module dependency structure clearly (except self-contained or cyclic-dependency exceptions).
