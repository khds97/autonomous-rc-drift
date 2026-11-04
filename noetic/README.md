# Autonomous RC Drift Car — ROS Noetic System

## 1. Project Overview

This branch contains the original ROS Noetic implementation of an autonomous rear-wheel-drive RC drift car.

The system is divided across three main parts:

- **RC car:** The physical vehicle, sensors, actuators, and embedded control hardware.
- **Raspberry Pi 4:** On-vehicle computer for sensor acquisition and camera operation.
- **Laptop:** Remote computer for visual-inertial odometry, vehicle-state processing, control, and learning-based policy development.

The Raspberry Pi and laptop communicate through a ROS network. The laptop sends steering and throttle commands to the vehicle through a USB-connected STM32 controller.

The overall design separates on-vehicle sensing from higher-level processing and control.

## 2. System Architecture

```text
                       ROS network
       ┌──────────────────────────────────────────┐
       │                                          │
       ▼                                          ▼
┌──────────────────────┐                 ┌──────────────────────┐
│ Raspberry Pi 4       │                 │ Laptop               │
│ On-vehicle computer  │                 │ ROS Noetic           │
│                      │                 │                      │
│  OAK-D camera        │── camera / IMU ─▶│  VINS-Fusion         │
│  Stereo + IMU        │                 │  Loop Fusion         │
│                      │                 │  Odometry processing │
│  Wheel / tire        │── /sensors ─────▶│  DriftEnv            │
│  speed sensors       │                 │  DAgger / NN policy  │
│  RPM, current        │                 │                      │
│                      │                 │  USB serial          │
│  ROS sensor node     │                 │       │              │
└──────────────────────┘                 └───────┼──────────────┘
                                                 │
                                          Steering / throttle
                                                 │
                                                 ▼
                                      ┌──────────────────────┐
                                      │ STM32 controller     │
                                      │ USB serial interface │
                                      └──────────┬───────────┘
                                                 │
                                                 ▼
                                      ┌──────────────────────┐
                                      │ RC car actuators     │
                                      │ Steering / ESC       │
                                      └──────────────────────┘
```

**Data flow in brief**

1. The Raspberry Pi 4 acquires vehicle sensor measurements and publishes them as ROS messages.
2. The OAK-D stereo camera and IMU provide visual and inertial data.
3. The laptop processes the camera/IMU stream with VINS-Fusion and Loop Fusion to obtain vehicle odometry.
4. The laptop combines odometry, IMU, vehicle sensors, and control feedback for the vehicle-state representation.
5. The control process generates steering and throttle commands.
6. Commands are transmitted over USB serial to the STM32 controller, which interfaces with the RC car's actuators.

## 3. Hardware and Software Responsibilities

| Component | Main responsibility | Related code |
|---|---|---|
| RC car | Steering and motor actuation; vehicle-mounted sensors | Vehicle hardware and embedded controller |
| Raspberry Pi 4 | Read wheel/tire signals and current; publish sensor data; run camera driver | `RPi4/driftmodel` |
| Laptop | Camera/IMU processing, odometry, state construction, control and learning | `laptop/driftmodel` |
| STM32 | Receive steering/throttle commands over USB serial and interface with vehicle electronics | Accessed by laptop through `USBReceiver` |
| Custom ROS messages | Define the vehicle sensor data exchanged between nodes | `RPi4/driftmodel_msg`, `laptop/driftmodel_msg` |

## 4. Raspberry Pi 4 — On-Vehicle Sensing

The Raspberry Pi package is located at:

`RPi4/driftmodel/`

Its main responsibilities are:

- Initialize GPIO inputs through pigpio.
- Count encoder and hall-sensor transitions for wheel-related measurements.
- Measure tire PWM input.
- Read current measurements through an ADS1115 ADC.
- Publish the acquired values as the custom ROS message `driftmodel_msg/sensors`.
- Launch the OAK-D stereo-inertial camera node.

The sensor message contains:

| Field | Description |
|---|---|
| `rpm` | RPM measurement |
| `tire_pwm` | Tire-related PWM measurement |
| `rl_speed` | Rear-left wheel speed |
| `rr_speed` | Rear-right wheel speed |
| `current` | Current measurement |
| `header` | Timestamp and frame information |

The sensor publisher runs at approximately 34 Hz in the current implementation.

Related packages:

- `RPi4/driftmodel_msg/`: custom sensor message definition.
- `RPi4/driftmodel/launch/driftmodel.launch`: launches the sensor node and stereo-inertial camera node.

## 5. Laptop — Perception, State Processing and Control

The laptop package is located at:

`laptop/driftmodel/`

The launch configuration starts the camera image republishers, RViz, VINS-Fusion, Loop Fusion, and supporting control/pose nodes.

The learning environment in `src/driftmodel_env.py` subscribes to synchronized ROS data:

- `/loop_fusion_node/odometry_rect`
- `/sensors`
- `/stereo_inertial_publisher/imu`
- `/ctrl_val`

These inputs are combined into a 21-dimensional vehicle-state vector for the learning-based controller. The action vector consists of steering, forward throttle, and brake throttle.

The detailed state representation, neural-network architecture, DAgger workflow, and code-review notes are documented separately in the [Drift Model README](laptop/driftmodel/README.md).

## 6. Laptop-to-Vehicle Command Interface

The laptop communicates with the STM32 controller through a USB serial connection.

The current `USBReceiver` implementation:

- Opens the serial device `/dev/ACMSTM32`.
- Uses a baud rate of 115200.
- Sends steering and throttle values as newline-separated text.
- Receives steering, throttle, mode, and punishment feedback from the serial stream.

This creates the command path:

```text
Laptop control process
        │
        ▼
USBReceiver
        │ USB serial
        ▼
STM32 controller
        │
        ▼
RC car steering / motor control
```

The STM32 is therefore the low-level interface between the laptop's ROS control software and the vehicle electronics.

## 7. Repository Structure

```text
noetic/
├── README.md
├── RPi4/
│   ├── driftmodel/
│   │   ├── include/       # Sensor and hardware interface headers
│   │   ├── launch/        # Raspberry Pi launch configuration
│   │   └── src/           # GPIO, ADC, wheel-speed and ROS publisher code
│   └── driftmodel_msg/    # Vehicle sensor message definition
└── laptop/
    ├── driftmodel/
    │   ├── launch/        # Camera, odometry and visualization launch setup
    │   └── src/           # Environment, USB interface, model and DAgger code
    └── driftmodel_msg/    # Matching ROS message package
```

## 8. Development History and Scope

This branch preserves the original ROS Noetic system and its distributed Raspberry Pi/laptop architecture.

The camera configuration and onboard hardware arrangement were later redesigned as part of a new development direction, with software work moving toward ROS 2 Foxy. The Noetic implementation is retained as a reference for the earlier vehicle platform, sensor interface, communication flow, and learning-based drift-control experiments.

This document describes the architecture represented by the current repository. Hardware wiring, firmware behavior, network configuration, and runtime launch dependencies should be checked against the actual vehicle setup before reproducing the system.
