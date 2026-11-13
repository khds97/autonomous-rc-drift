# Autonomous RC Drift Car - ROS 2 Foxy

ROS 2 Foxy version of the 1/10-scale autonomous RC drift car. The architecture is the same as the ROS 1 Noetic prototype (sensor node on the Raspberry Pi 4, behavior cloning with DAgger on the laptop, STM32 board for the radio and motor commands). The code was ported and the problems found in the prototype were fixed along the way.

The Noetic implementation is on the `noetic` branch.

**Status:** ported and unit-tested on a PC. It has not been built with `colcon` or run on the car yet (see [Testing](#testing)).

## Layout

```
foxy/
├── controller/             STM32F401CCU6 firmware (USB CDC, ADC, I2C, MCP4725 DAC)
├── RPi4/
│   ├── driftmodel_msg/     msg/Sensors.msg
│   ├── driftmodel/         sensor node (C++, pigpio, ADS1115), launch/
│   └── camera_publisher/   dual USB camera publisher (V4L2, OpenCV), launch/
└── laptop/
    ├── driftmodel_msg/     msg/{Sensors,CtrlVal,PoseFlag}.msg, srv/Flag.srv
    └── driftmodel/         preprocess, model, usb_receiver, driftmodel_env,
                            train_agent, dagger, pose_flag, sync_sub; launch/, demo.rviz
```

The `controller/` directory contains the STM32F401CCU6 firmware used for the RC transmitter interface. This is the controller firmware carried over from the Noetic setup; the ROS 2 port mainly changes the Raspberry Pi and laptop software. The controller communicates with the laptop through USB CDC, reads the transmitter's steering and throttle signals through ADC, and uses two MCP4725 DACs to output analog steering and throttle signals. GPIO inputs select between manual transmitter control and laptop commands.

The STM32 project includes its CubeMX configuration, application source, HAL/CMSIS drivers, USB CDC middleware, and the MCP4725 driver. IDE workspace metadata and compiled Debug output are excluded.

Differences from the Noetic layout:

- `msg/` and `srv/` are at the package root and the names are CamelCase (required by ROS 2).
- The laptop-side USB module is Python (`usb_receiver.py`) instead of C++, so there is no `include/` on the laptop.
- Launch files are Python (`launch/*.py`). `demo.rviz` is a minimal rviz2 config (grid and odometry).
- A separate `camera_publisher` package is included under `RPi4/` for the fixed and steering-linked rotating cameras. It publishes two grayscale image topics.
- OpenVINS/VINS integration is not currently included. The camera package does not publish IMU or camera calibration data, and the two camera streams are not hardware-synchronized.
- The recorded data and trained models were not carried over.

## Device-specific setup

This repository contains the laptop, Raspberry Pi, and STM32 controller files in one Git repository. Each device can use Git sparse checkout to populate only the directory it needs. The instructions below use a normal clone followed by sparse checkout; this avoids the partial-clone options and keeps the commands straightforward.

The commands below assume Ubuntu 20.04, ROS 2 Foxy, Python 3.8, and a fresh checkout. Run the laptop commands on the laptop and the Raspberry Pi commands on the Raspberry Pi. The laptop and Raspberry Pi are separate machines, so each one should have its own clone.

### Laptop

Clone only the laptop-side ROS 2 packages:

```bash
cd ~
git clone -b foxy https://github.com/khds97/autonomous-rc-drift.git

cd ~/autonomous-rc-drift
git sparse-checkout init --cone
git sparse-checkout set foxy/laptop
```

Create the ROS 2 workspace and link the two packages:

```bash
mkdir -p ~/foxy_ws/src

ln -s ~/autonomous-rc-drift/foxy/laptop/driftmodel_msg \
  ~/foxy_ws/src/driftmodel_msg
ln -s ~/autonomous-rc-drift/foxy/laptop/driftmodel \
  ~/foxy_ws/src/driftmodel
```

Install the Python dependencies and build:

```bash
pip3 install torch scikit-learn joblib pynput pyserial numpy

cd ~/foxy_ws
colcon build
source install/setup.bash
```

Launch the laptop node:

```bash
ros2 launch driftmodel driftmodel.launch.py \
  serial_port:=/dev/ACMSTM32 use_rviz:=true
```

### Raspberry Pi 4

Clone only the Raspberry Pi-side ROS 2 packages:

```bash
cd ~
git clone -b foxy https://github.com/khds97/autonomous-rc-drift.git

cd ~/autonomous-rc-drift
git sparse-checkout init --cone
git sparse-checkout set foxy/RPi4
```

Create the ROS 2 workspace and link the two packages:

```bash
mkdir -p ~/foxy_ws/src

ln -s ~/autonomous-rc-drift/foxy/RPi4/driftmodel_msg \
  ~/foxy_ws/src/driftmodel_msg
ln -s ~/autonomous-rc-drift/foxy/RPi4/driftmodel \
  ~/foxy_ws/src/driftmodel
```

Build the packages:

```bash
cd ~/foxy_ws
colcon build
source install/setup.bash
```

Start the pigpio daemon and launch the sensor node:

```bash
sudo pigpiod
ros2 launch driftmodel driftmodel.launch.py
```

### Updating an existing sparse checkout

After new commits are pushed to the `foxy` branch, update the checkout on the relevant device. The sparse-checkout selection is retained:

```bash
cd ~/autonomous-rc-drift
git pull origin foxy

cd ~/foxy_ws
colcon build
source install/setup.bash
```

The sparse-checkout selection is retained, so the laptop continues to populate `foxy/laptop` and the Raspberry Pi continues to populate `foxy/RPi4`.

### STM32 controller

The `foxy/controller/` directory contains STM32CubeIDE firmware, not a ROS 2 package. It is kept in the same repository for version control, but it is not added to the ROS 2 workspace with `colcon`. Open the STM32 project in STM32CubeIDE and build/flash it separately.

## Running the learning code

DAgger is started by the `pose_flag` node (it runs `dagger.py`) or directly with `ros2 run driftmodel dagger.py`.

Offline training: `ros2 run driftmodel train_agent.py <model_path> --data_dir <dir>`.

Data and models go to `~/driftmodel/data` and `~/driftmodel/dagger_models` by default.

Sensor node parameters: `publish_rate_hz` (34), `speed_window_s` (0.1), `motor_counts_per_rev` (6), `magnets_per_wheel` (8), `rpm_timeout_s` (0.5).

## Problems found in the Noetic prototype and what changed

I reviewed the Noetic code together with its recorded dataset (12,401 samples, 4 episodes). "Measured" means computed from the dataset, "code" means read from the source only.

### Data and learning

- **`tire_pwm` leaks the steering label** (measured). It is correlated with the steering action (-0.64). In a 5-fold temporal cross-validation the steering MSE goes from 0.069 to 0.167 without it, close to the 0.19 of a predictor that always outputs the mean. If this pin is the steering PWM that is applied to the car, the action leaks into the state. *Now:* excluded from the network input by default, `--use_tire_pwm` turns it back on. Whether it is an input or an output signal is still open.
- **Throttle was split into two channels** (measured). `forward` and `brake` come from one voltage split at about 1.8 V, and the inactive channel is stored as 0, which is not a real value. *Now:* one throttle value, so the action has 2 values (steer, throttle).
- **IMU acceleration was never recorded** (measured). `ax/ay/az` were filled from the angular velocity. *Now:* fixed mapping; the old dataset cannot be used and the loaders reject it.
- **State and action were misaligned** (code). The action was read after a 0.1 s sleep, so it was paired with a state about 0.2 s older. *Now:* states and actions are joined by time stamp with `ApproximateTimeSynchronizer`.
- **Almost no DAgger relabeling** (measured). In autonomous mode the recorder published a constant action, so only 162 samples (1.3%) carry that label and the data is almost all manual driving. *Now:* the operator input is published in both modes (this assumes the STM32 keeps reporting the sticks in autonomous mode; `--exclude_autonomous` drops those samples if it does not).
- **The dataset was overwritten at every start** (code), so aggregation only worked inside one run. *Now:* new samples are appended.
- **Training setup** (measured). SGD without shuffling gave a validation error about equal to the mean predictor. Adam lowered the mean error from about 0.44 to 0.31 in the same cross-validation. *Now:* Adam, shuffled batches, and the result is reported next to two baselines (training mean, and "repeat the previous action", steering MSE 0.028).
- **Scaler not saved with the model** (code). *Now:* `model_N.pth` and `model_N_scaler.pkl` are saved together, and actions are de-normalized at inference.

### Sensing

- **Wheel speed** (code, and measured for the coarse values). The counters were cleared every publish cycle (about 34 Hz) but the speed was computed only once per 100 ms, so it was probably about 3 times too low and repeated. `rpm` had only 28 distinct values and `rl_speed`/`rr_speed` only 14. *Now:* each wheel speed comes from the time between two magnet passes (8 magnets, one hall sensor per wheel, falling edges only) and is 0 after `rpm_timeout_s` without a pulse. Pins: FL 19, FR 5, RL 13, RR 6. The motor speed is a rate over a sliding window of real elapsed time.
- **No direction on the wheels** (code, measured). The callbacks only counted up. With one sensor per wheel the direction can not be measured, so wheel speeds are unsigned.
- **ADC scale** (code). The ADS1115 was configured for +-2.048 V but converted with the +-6.144 V step (current about 3 times too high), and a small negative reading turned into about 12 V. *Now:* correct scale, signed value.
- **Unsynchronized counters** (code). The pigpio callback thread and the main loop shared plain variables. *Now:* `std::atomic` for the motor counter, the wheel pulse time/period and the PWM width.
- **Wrong object in the pigpio callbacks** (code). `driver.cpp` passed the `driver` pointer as callback user data, but the callbacks cast it to `wheelSpeed*` / `pwm*`. *Now:* each callback gets its own object.

### Command interface and safety

- **Serial parser** (code). It read integers into `bool` variables with `%d` (writes outside the variable) and used only the first line of the received bytes. *Now:* the newest complete line is parsed.
- **Command limits** (code). The `else if` chain clamped only one value per call and had a typo (`throttle = 0`). *Now:* every value is clamped to 0..3.3 V.
- **No stop command between episodes** (code). While the model trains, the last command stayed active unless the firmware has a timeout. *Now:* a neutral command is sent when an episode ends or the program stops. A firmware-side timeout is still recommended.
- **`driver::init()` / `deinit()`** returned `int` without a return value. *Now:* they return status codes.

### Reproducibility

- Absolute paths such as `/home/ubuntu/ros_noetic_ws/...` were hard-coded in the DAgger script, the launcher node and the launch file. Now data and model directories are command-line options (default `~/driftmodel/`).
- Results should be reported with the baselines above and with repeatable driving tests, not only offline error.

## Preprocessing

16 of the 21 state values are standardized with `StandardScaler`, and both action values are standardized and de-normalized at inference. `pz` and the quaternion are left unscaled because their spread is small or already bounded. In a temporal cross-validation the choice for these columns changed the error by less than the seed-to-seed spread, so it is a design choice, not a measured gain. The physical meaning of the `pz` axis was not checked.

## Notes

- Wheel speeds are `60e6 / (period_us * magnets_per_wheel)`. With 8 magnets and a 0.5 s timeout, speeds below about 15 rpm read 0. A single period is not averaged, so uneven magnet spacing shows up as jitter.
- `Sensors.msg` also carries `fl_speed` / `fr_speed`, but the model still uses only `rl_speed` / `rr_speed` (21 features).
- `motor_counts_per_rev = 6` is an assumption (6 steps per electrical revolution, not multiplied by the pole pairs). Check it against the motor.
- The hall pins use the internal pull-up.
- The PWM measurement is on GPIO 21 (`TIRE`). Check this pin on the car.
- `step()` waits up to 1.0 s (`sample_timeout`) for new data, and the DAgger loop stops after 20 timeouts in a row.

## Testing

| Part | Status |
|---|---|
| Wheel speed and motor hall decoding (`wheelSpeed.hpp`) | Unit tests on a PC with synthetic pulses: 600-3000 rpm, tick wrap, timeout, late pulse, 4 wheels at different speeds, zero on stop |
| `driver`, `pwm`, `ADS1115`, `I2Cdev` | Compiled with `-Wall -Wextra`. `driver` was linked and run against a stub pigpio |
| Preprocessing, training, DAgger loop, USB parser, environment | Python tests with a fake serial port, a fake environment and stubbed ROS modules. Training was run on the old Noetic data converted to the new format |
| `pubSensorData.cpp`, `driftmodel_node.cpp`, rclpy runtime, launch files, `demo.rviz` | Checked for syntax and structure only. Not built or run with ROS 2 |
| Hardware (pigpio, ADS1115, STM32) | Not tested |

## Related

- Repository overview: [`../README.md`](../README.md)
- Noetic implementation: `noetic` branch, `noetic/laptop/driftmodel/README.md`
