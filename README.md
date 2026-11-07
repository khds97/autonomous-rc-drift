# Autonomous RC Drift Car

A 1/10-scale autonomous RC drift car project focused on vehicle-state estimation, motion control, and learning-based autonomous drifting.

## Project Overview

The goal of this project is to develop an RC car that can perform controlled drifting maneuvers autonomously. The system combines vehicle sensor data, ROS-based communication, and learning-based control.

The repository contains development work across different ROS environments:

- **Foxy** (this branch): ROS 2 version of the sensor node and the learning pipeline.
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
    ├── RPi4/       # sensor node and messages for the Raspberry Pi 4
    └── laptop/     # learning pipeline, DAgger loop and messages for the laptop
```

The ROS 1 Noetic implementation is in the `noetic` branch (`noetic/`).

## Project Documentation

- Foxy version, build instructions and the list of fixes: [`foxy/README.md`](foxy/README.md)
- Noetic drift-control learning pipeline (state/action representation, ROS environment, neural-network model, DAgger workflow): `noetic/laptop/driftmodel/README.md` in the `noetic` branch

## Project Status

This project is under development. The Foxy code has been unit-tested on a PC but not yet built with `colcon` or run on the car.
