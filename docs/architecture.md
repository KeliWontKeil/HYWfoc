# Architecture Overview

## System Design

The GD32F303CC FOC project implements a real-time motor control system with the following key components:

### Core Architecture
- **Microcontroller**: GD32F303CC (ARM Cortex-M4, 120MHz)
- **Real-time Framework**: Timer-based multi-rate task scheduling
- **Control Algorithm**: Low-speed sensored FOC complete for current stage (torque/current/position/speed path available; precise current-control tuning remains next focus)
- **Feedback**: Current sensing + position encoder

### Software Layers

#### Level 1: Application Layer
- `foc_app.c`: public app entry for `main.c` (`FOC_App_Init/Start/Loop`)
- `main.c`: thin bootstrap, calls only level-1 API

#### Level 2: Algorithm Layer
- `foc_control.c`: FOC runtime control algorithms
- `foc_control_init.c`: startup calibration and motor init algorithm flow
- `control_scheduler.c`: task scheduling logic
- `uart_debug.c`: debug output policy and formatting
- Protocol/Task/State modules (planned): parser, command executor, status monitor

#### Level 3: Advanced Peripheral Layer
- `sensor.c`: converts raw peripheral data to structured sensor outputs
- `svpwm.c`: converts voltage/vector command to structured PWM duty request
- Serial parser adapter (planned): parses stream/buffer to structured protocol frames

#### Level 4: Peripheral Layer (Utilities)
- **ADC**: Current sampling with DMA (PA6/PA7 synchronous, 12-bit resolution)
  - Dual-channel ADC for phase current measurement
  - DMA transfer for low-latency data acquisition
  - Calibration and offset compensation
- **PWM**: 3-phase complementary PWM generation with dead time
  - Timer-based PWM with configurable frequency and duty cycle
  - Dead time insertion for safe switching
  - Brake functionality for emergency stop
- **USART**: Dual serial communication with RX interrupt + DMA TX path
  - Configurable baud rate and data format
  - Circular buffer for reliable data reception
  - DMA-based non-TBE transmission and interrupt-based reception
- **I2C**: Hardware I2C driver for sensor communication
  - Master mode operation with clock stretching support
  - Error handling and recovery mechanisms
  - Multi-device addressing capability
- **Timer**: Hardware timer management (Timer0/1/2/3)
  - Configurable prescaler and auto-reload values
  - Interrupt generation for periodic tasks
  - Input capture and output compare modes
- **LED**: Status indication GPIO control
  - Multi-color LED status signaling
  - PWM dimming capability
  - Fault indication and user feedback
- **AS5600**: Magnetic encoder driver with angle conversion
  - 12-bit resolution angle measurement
  - I2C interface with automatic gain control
  - Zero position calibration and offset compensation
  - Native radian conversion API for control-path consistency

#### Special Dependency Layer
- Purpose: unify cross-layer APIs, shared structs, parameter configuration, and algorithm feature-cut macros.
- Current candidates:
  - `foc_platform_api.[ch]` (unified upper API)
  - `foc_shared_types.h` (shared public structures)
  - config headers (planned): control/debug/protocol/task macro switches
- Contract:
  - L1/L2/L3 must access L4 only via this layer.
  - L4 must not depend on L1/L2/L3.

#### Hardware Abstraction Layer
- GD32F30x standard peripheral library
- CMSIS-Core for Cortex-M4

## Timing Architecture

### Task Scheduling
The system uses a hierarchical timing framework:

```text
TIMER1 (1kHz)
├── Timer1 IRQ -> Timer1_IRQHandler_Internal() -> ControlScheduler_RunTick()
├── 1kHz callback slot: control loop + sensor refresh
├── 100Hz callback slot: medium-rate extension slot
├── 10Hz callback slot: slow monitoring slot
└── 1Hz callback slot: status updates (LED heartbeat)

TIMER2 (24kHz master)
├── Drives TIMER0 slave synchronization for PWM base timing
├── Provides trigger chain reference for sampling timing
└── Drives SVPWM interpolation callback for high-rate duty update

TIMER3 (compare trigger)
└── Generates ADC external trigger event (current sampling phase alignment)
```

### Execution Budget
- **1kHz tasks**: < 1ms total execution time
- **100Hz tasks**: < 10ms execution time
- **10Hz tasks**: < 100ms execution time
- **1Hz tasks**: < 1s execution time

## Data Flow

### Control Loop
```
ADC Samples → Current Calculation/Filtering → FOC Control API (torque / current-loop / angle-loop / speed-loop entry) → PWM Duty Cycle
    ↑                                                       ↓
Position Encoder ←───────────────────────────────────── Motor Driver
```

### Sensor-Control Boundary
- Sensor module is responsible for acquisition/filtering only.
- Control module accepts sampled phase currents and mechanical angle as API inputs.
- Mechanical-to-electrical angle conversion is handled in FOC control layer using calibrated zero, direction, and pole pairs.

### Startup Calibration Flow
```
FOC_MotorInit()
  └─ FOC_CalibrateElectricalAngleAndDirection()
       ├─ Lock electrical angle at 0 and sample mechanical angle (zero electrical reference)
       └─ Step electrical angle forward with settle+sample at each step
            ├─ Estimate direction from accumulated mechanical increment sign
            └─ Estimate pole pairs from accumulated electrical/mechanical angle ratio
```

### Communication Flow
```
USART1 ←→ Debug Output Channel (DMA TX)
USART2 ←→ Secondary Serial Channel (DMA TX + RX IRQ callback)
I2C ←→ AS5600 Encoder
```

## Memory Map

### RAM Usage (Current: ~1.7KB/96KB)
- Static variables and buffers
- DMA buffers for ADC
- Stack and heap

### Flash Usage (Current: ~9KB/256KB)
- Application code
- Peripheral drivers
- Vendor libraries

## Module Dependencies

```
main.c
└── L1: foc_app.c

L1: foc_app.c
└── L2/L3 modules via public headers

L2: foc_control.c / foc_control_init.c / control_scheduler.c / uart_debug.c
└── Special layer API/types/macros

L3: sensor.c / svpwm.c / serial parser adapter(planned)
└── Special layer API/types/macros

Special layer: foc_platform_api.c + foc_shared_types.h + config headers(planned)
└── L4 Utilities only

L4: Utilities/*
└── chip-specific low-level drivers only

gd32f30x_it.c
├── adc internal IRQ handler
├── usart1 internal IRQ handler
├── usart2 internal IRQ handler
├── timer1 internal IRQ handler
├── timer2 internal IRQ handler
└── dma internal IRQ handler
```

## Current Gap Check (against target layering)
- `foc_platform_api.h` currently includes multiple L4 headers directly, which leaks peripheral symbols to upper layers.
- `svpwm` PWM calls are routed through special layer API; direct `pwm.h` dependency has been removed.
- `foc_control.h` currently includes `foc_platform_api.h` and `svpwm.h`; algorithm-layer public header exposure should be narrowed to shared types and algorithm APIs.
- ISR glue (`gd32f30x_it.h`) includes utility headers directly; this is acceptable only if treated as special-layer boundary file.

## Safety Considerations
- Interrupt-safe data handling
- Watchdog timer integration
- Error recovery mechanisms
- Resource usage monitoring

## Extension Points
- Additional sensor interfaces
- Communication protocols (CAN, USB)
- Advanced control algorithms
- Data logging capabilities