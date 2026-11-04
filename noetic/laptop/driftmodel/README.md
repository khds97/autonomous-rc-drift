# Drift Control with Imitation Learning

## 1. Overview

This package implements a learning-based control pipeline for an autonomous RC drift car. It connects ROS vehicle and sensor topics to a Gym-style environment, records expert control demonstrations, and trains a neural network to predict control actions from vehicle states.

The learning workflow uses **imitation learning** with **Dataset Aggregation (DAgger)**. The expert driver provides reference actions while the current policy is executed; the collected state/action pairs are then used to train the next policy iteration.

## 2. System Architecture

```text
                  ROS topics
       ┌──────────────────────────────┐
       │ Odometry                     │
       │ IMU                          │
       │ Vehicle sensors              │
       │ Control values (expert input)│
       └──────────────┬───────────────┘
                      │
                      ▼
             ┌─────────────────┐
             │  DriftEnv       │
             │ driftmodel_env  │
             │                 │
             │ State: 21 dims  │
             │ Action: 3 dims  │
             │ Reward / Done   │
             └────────┬────────┘
                      │
             ┌────────┴─────────┐
             │                  │
             ▼                  ▼
      Expert action        Current policy
       actionCB()          Neural network
             │                  │
             └────────┬─────────┘
                      │
                DAgger mixing
                      │
                      ▼
             RC control command
                      │
                      ▼
              USB transmitter

      Recorded state/action samples
                      │
                      ▼
              data_dagger.pkl.gzip
                      │
                      ▼
              train_agent.py
                      │
                      ▼
              Trained model (.pth)
                      │
                      └──── next DAgger iteration
```

## 3. Environment and Data Representation

The `DriftEnv` class in `src/driftmodel_env.py` acts as the interface between ROS and the learning code.

### State (21 dimensions)

The observation vector is assembled in the following order:

| Index | Feature | Source |
|---:|---|---|
| 0–2 | Position: `px, py, pz` | Odometry |
| 3–6 | Orientation: `qw, qx, qy, qz` | Odometry |
| 7–9 | Linear velocity: `vx, vy, vz` | Odometry |
| 10–12 | Acceleration fields: `ax, ay, az` | IMU callback |
| 13–15 | Angular velocity fields: `gx, gy, gz` | IMU callback |
| 16 | RPM | Vehicle sensor message |
| 17 | Tire PWM | Vehicle sensor message |
| 18–19 | Rear-left / rear-right speed | Vehicle sensor message |
| 20 | Current | Vehicle sensor message |

The current observation vector is:

```python
[
    px, py, pz,
    qw, qx, qy, qz,
    vx, vy, vz,
    ax, ay, az,
    gx, gy, gz,
    rpm, tire_pwm, rl_speed, rr_speed, current
]
```

### Action (3 dimensions)

The action vector contains:

```python
[steer, forward_throttle, brake_throttle]
```

The expert action is obtained through `DriftEnv.actionCB()`. The environment sends the selected steering and throttle command to the vehicle through the USB receiver.

## 4. Neural Network

The model is defined in `src/model.py` using PyTorch.

| Layer | Dimensions | Activation |
|---|---:|---|
| Input | 21 | — |
| Fully connected 1 | 21 → 11 | ReLU |
| Fully connected 2 | 11 → 5 | ReLU |
| Output | 5 → 3 | Linear |

The network maps a 21-dimensional vehicle state to a 3-dimensional control action.

## 5. DAgger Training Workflow

The main iteration loop is implemented in `src/dagger.py`.

1. Initialize the ROS-connected `DriftEnv`.
2. Load the current policy model.
3. Read the current vehicle state.
4. Obtain the expert control action.
5. Predict an action using the current neural-network policy.
6. Combine expert and policy actions using a mixing coefficient:

   ```python
   pi = beta * expert_action + (1 - beta) * policy_prediction
   ```

7. Apply the selected action and record state, action, next state, reward, and terminal information.
8. At the end of an episode, save the aggregated dataset.
9. Train the next model with `train_agent.py`.
10. Repeat with the updated policy.

The mixing coefficient is scheduled as `beta_i ** model_number` in the current implementation, gradually reducing the expert contribution as the iteration number increases.

## 6. Dataset Format

The collected dataset is stored as a gzip-compressed Python pickle file:

`src/data/data_dagger.pkl.gzip`

It contains:

| Key | Description |
|---|---|
| `state` | Vehicle state at the current step |
| `action` | Expert action recorded for that state |
| `next_state` | Vehicle state after the action |
| `reward` | Environment reward |
| `terminal` | Episode termination flag |

## 7. Training Implementation

`src/train_agent.py` loads the dataset and trains the policy.

- Loads state and action arrays from the compressed dataset.
- Splits samples into training and validation portions in their existing order.
- Fits a `StandardScaler` on training states and applies it to training and validation states.
- Trains the network using Mean Squared Error (MSE) loss and SGD with weight decay.
- Saves the trained PyTorch model as a `.pth` file.

## 8. Current Implementation Notes

This document describes the current code structure; it does not imply that every part of the pipeline has been fully validated. The code review identified several issues to address:

- In `driftmodel_env.py`, both `ax/ay/az` and `gx/gy/gz` are assigned from `imu.angular_velocity`. Thus, the fields named acceleration currently do not contain linear-acceleration measurements.
- `train_agent.read_data()` scales training and validation states, but does not return the fitted scaler. The DAgger inference path currently passes the raw state to the model. Training and inference preprocessing therefore need to be made consistent.
- The validation arrays are passed to `train_model()`, but the current training loop does not calculate validation loss or use validation data for model selection.
- The standalone `train_agent.py` entry point calls `read_data(..., frac=0.9)`, while the function default and the DAgger call use `frac=0.1`. The intended split should be confirmed.

These are improvement points for subsequent implementation and testing, rather than claims about measured model performance.

## 9. Planned Improvements

- Correct and verify IMU field assignments.
- Save and reuse preprocessing parameters during inference.
- Add validation metrics and learning curves.
- Evaluate action scaling and output constraints.
- Compare policy performance across DAgger iterations.
- Document repeatable driving tests and quantitative results.
