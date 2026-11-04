# Autonomous RC Drift Car

A 1/10-scale autonomous RC drift car project focused on vehicle-state estimation, motion control, and learning-based autonomous drifting.

## Project Overview

The goal of this project is to develop an RC car that can perform controlled drifting maneuvers autonomously. The system combines vehicle sensor data, ROS-based communication, and learning-based control.

The repository contains development work across different ROS environments:

- **Foxy:** ROS 2 development.
- **Noetic:** ROS 1 development, including the imitation-learning-based drift control package.

## Main Objectives

- Acquire vehicle motion and sensor data.
- Send steering and throttle commands to the RC car.
- Collect expert driving demonstrations.
- Train a neural-network policy using imitation learning.
- Improve the policy iteratively with Dataset Aggregation (DAgger).

## Repository Structure

```text
.
└── foxy/       # ROS 2 (Foxy) development
```

The ROS 1 Noetic implementation is maintained in the `noetic` branch.

## Project Documentation

For the Noetic drift-control learning pipeline, including the state/action representation, ROS environment, neural-network model, and DAgger workflow, see the README in the `noetic/laptop/driftmodel` package.

## Project Status

This project is under development. The control pipeline, data preprocessing, training procedure, and evaluation methods are being progressively documented and improved.
