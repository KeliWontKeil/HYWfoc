# Architecture Overview

## System Design

The GD32F303CC FOC project implements a real-time motor control system with the following key components:

### Core Architecture
- **Microcontroller**: GD32F303CC (ARM Cortex-M4, 120MHz)
- **Real-time Framework**: Timer-based multi-rate task scheduling
- **Control Algorithm**: Torque-control FOC with position-loop framework extension
- **Feedback**: Current sensing + position encoder

### Software Layers

#### Application Layer
- `main.c`: System initialization and main loop
- `timer1_algorithm.c`: Multi-rate task scheduler
- `foc_control.c`: FOC voltage synthesis, startup calibration, torque-control API, and position-loop entry
- `sensor.c`: Current and angle acquisition/filter path

#### Driver Layer (Utilities/)
- **ADC**: Current sampling with DMA (PA6/PA7 synchronous, 12-bit resolution)
  - Dual-channel ADC for phase current measurement
  - DMA transfer for low-latency data acquisition
  - Calibration and offset compensation
- **PWM**: 3-phase complementary PWM generation with dead time
  - Timer-based PWM with configurable frequency and duty cycle
  - Dead time insertion for safe switching
  - Brake functionality for emergency stop
- **USART**: Serial communication with ring buffers and interrupt handling
  - Configurable baud rate and data format
  - Circular buffer for reliable data reception
  - Interrupt-driven transmission and reception
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

#### Hardware Abstraction Layer
- GD32F30x standard peripheral library
- CMSIS-Core for Cortex-M4

## Timing Architecture

### Task Scheduling
The system uses a hierarchical timing framework:

```
TIMER1 (1kHz)
├── Algorithm_1kHz_Task() - FOC open-loop step + sensor update
├── Algorithm_100Hz_Task() - Medium-rate extension slot
├── Algorithm_10Hz_Task() - Slow monitoring slot
└── Algorithm_1Hz_Task() - Status updates (LED heartbeat)

TIMER2 (24kHz master)
├── Drives TIMER0 slave synchronization for PWM base timing
└── Provides trigger chain reference for sampling timing

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
ADC Samples → Current Calculation/Filtering → FOC Control API (torque / current-loop / angle-loop entry) → PWM Duty Cycle
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
USART1 ←→ Debug Output
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
├── systick.c
├── LED.c
├── USART1.c
├── TIMER1.c
├── TIMER2.c
├── SVPWM.c
├── foc_control.c
└── sensor.c

timer1_algorithm.c
├── TIMER1.c
└── DWT (for timing)

sensor.c
├── ADC.c
├── TIMER3.c
├── AS5600.c
└── I2C0.c
```

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