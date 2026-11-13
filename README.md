# Autonomous RC Drift Car

A 1/10-scale autonomous RC drift car project focused on vehicle-state estimation, motion control, and learning-based autonomous drifting.

## Project Overview

The goal of this project is to develop an RC car that can perform controlled drifting maneuvers autonomously. The system combines vehicle sensor data, ROS-based communication, and learning-based control.

The repository contains development work across different ROS environments:

- **Foxy** (this branch): ROS 2 version of the Raspberry Pi sensor packages and the laptop-side learning pipeline.
- **Noetic** (`noetic` branch): ROS 1 prototype, including the imitation-learning-based drift control package.

## Main Objectives

- Acquire vehicle motion and sensor data.
- Send steering and throttle commands to the RC car.
- Collect expert driving demonstrations.
- Train a neural-network policy using imitation learning.
- Improve the policy iteratively with Dataset Aggregation (DAgger).

## Repository Structure

```text
.
└── foxy/
    ├── controller/  # STM32 transmitter interface firmware
    ├── RPi4/        # vehicle sensors, cameras, IMU and ROS messages
    └── laptop/      # learning pipeline, DAgger loop and ROS messages
```

The ROS 1 Noetic implementation is in the `noetic` branch (`noetic/`).

## Project Documentation

- Foxy version, device-specific setup, package details, testing status and known changes: [`foxy/README.md`](foxy/README.md)
- Raspberry Pi 4 packages, including camera and IMU setup: [`foxy/RPi4/README.md`](foxy/RPi4/README.md)
- Laptop packages and setup: [`foxy/laptop/README.md`](foxy/laptop/README.md)
- Noetic drift-control learning pipeline (state/action representation, ROS environment, neural-network model, DAgger workflow): `noetic/laptop/driftmodel/README.md` in the `noetic` branch

## Project Status

The Raspberry Pi 4 camera and IMU packages have been built with `colcon`, and their ROS 2 topics have been verified publishing on the Pi. The IMU bias calibration and further sensor checks are in progress. The remaining vehicle sensor hardware, STM32 controller, and complete laptop-to-car integration have not yet been verified as an integrated system.

See [Foxy Testing](foxy/README.md#testing) for component-level test details.
