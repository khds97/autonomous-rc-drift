# Autonomous RC Drift Control

## Project Overview

This project explores autonomous drifting control for a rear-wheel-drive RC car using imitation learning.

The system collects vehicle-state and control-action data while the car is driven by an expert policy. A neural network is trained to predict steering, throttle, and braking commands from the vehicle state. The project uses Dataset Aggregation (DAgger) to iteratively collect additional data from the learner's driving experience and improve the learned control policy.

The long-term goal is to develop a control policy that can perform stable and repeatable drifting maneuvers autonomously.

## Key Components

- **Vehicle state acquisition:** Collect vehicle and sensor information through ROS.
- **Expert data collection:** Record vehicle states and corresponding control actions.
- **Imitation learning:** Train a neural network to predict control actions from vehicle states.
- **DAgger:** Iteratively aggregate driving data and retrain the policy.

## Project Status

This project is under active development. Data preprocessing, model training, and evaluation are being reviewed and improved.

## Planned Documentation

- System architecture
- Vehicle state and action definitions
- Data collection process
- DAgger training workflow
- Model architecture and training configuration
- Evaluation results and future improvements
