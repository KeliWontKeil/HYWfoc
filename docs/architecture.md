# Architecture Overview

## System Design

The GD32F303CC FOC project implements a real-time motor control system with the following key components:

### Core Architecture
- **Microcontroller**: GD32F303CC (ARM Cortex-M4, 120MHz)
- **Real-time Framework**: Timer-based multi-rate task scheduling
- **Control Algorithm**: Field Oriented Control for 3-phase motors
- **Feedback**: Current sensing + position encoder

### Software Layers

#### Application Layer
- `main.c`: System initialization and main loop
- `timer1_algorithm.c`: Multi-rate task scheduler
- User control logic (to be implemented)

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
- **Timer**: Hardware timer management (Timer1/Timer2 with parameterized initialization)
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

#### Hardware Abstraction Layer
- GD32F30x standard peripheral library
- CMSIS-Core for Cortex-M4

## Timing Architecture

### Task Scheduling
The system uses a hierarchical timing framework:

```
TIMER1 (1kHz)
├── Algorithm_1kHz_Task() - Fast control loops
├── Algorithm_100Hz_Task() - Medium-rate tasks
├── Algorithm_10Hz_Task() - Slow monitoring
└── Algorithm_1Hz_Task() - Status updates
```

### Execution Budget
- **1kHz tasks**: < 1ms total execution time
- **100Hz tasks**: < 10ms execution time
- **10Hz tasks**: < 100ms execution time
- **1Hz tasks**: < 1s execution time

## Data Flow

### Control Loop
```
ADC Samples → Current Calculation → FOC Algorithm → PWM Duty Cycle
    ↑                                                       ↓
Position Encoder ←───────────────────────────────────── Motor Driver
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
├── PWM.c
├── TIMER1.c
├── I2C.c
├── AS5600.c
└── ADC.c

timer1_algorithm.c
├── TIMER1.c
└── DWT (for timing)
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