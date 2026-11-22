# Raspberry Pi 4

ROS 2 Foxy packages for the Raspberry Pi 4.

## Packages

| Package | Description |
|---|---|
| `driftmodel_msg` | Custom `Sensors` message definitions |
| `driftmodel` | Vehicle sensor data node |
| `sensor_publisher` | Publishes images from the fixed and rotating cameras and accelerometer and gyroscope data from the ISM330DHCX |

## Requirements

- Raspberry Pi 4
- Ubuntu 20.04
- ROS 2 Foxy
- Git
- Colcon

## Get the Raspberry Pi packages

The repository uses Git sparse checkout so each device can work with its own part of the `foxy` branch.

Run the following commands on the Raspberry Pi:

```bash
cd ~

git clone -b foxy https://github.com/khds97/autonomous-rc-drift.git

cd ~/autonomous-rc-drift
git sparse-checkout init --cone
git sparse-checkout set foxy/RPi4
```

The Raspberry Pi packages are available under:

```
foxy/RPi4/
```

Check the selected directory with:

```bash
git sparse-checkout list
```

If you already cloned the full repository, you do not need to clone it again. From the repository directory, run:

```bash
git sparse-checkout init --cone
git sparse-checkout set foxy/RPi4
```

Cone-mode sparse checkout also keeps files directly inside the repository root and the `foxy/` parent directory. The device-specific packages themselves remain under `foxy/RPi4/`.

## Install dependencies

Make sure ROS 2 Foxy is installed, then run:

```bash
source /opt/ros/foxy/setup.bash
cd ~/autonomous-rc-drift/foxy/RPi4

rosdep update
rosdep install --from-paths . --ignore-src -r -y
```

If `rosdep` is not installed:

```bash
sudo apt update
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

## Build

Build the packages in the Raspberry Pi workspace:

```bash
cd ~/autonomous-rc-drift/foxy/RPi4
source /opt/ros/foxy/setup.bash

colcon build --symlink-install
```

Source the workspace after building:

```bash
source install/setup.bash
```

For each new terminal:

```bash
source /opt/ros/foxy/setup.bash
source ~/autonomous-rc-drift/foxy/RPi4/install/setup.bash
```

## Run

### Cameras and IMU

Start both publishers with one launch file:

```bash
ros2 launch sensor_publisher sensors.launch.py
```

See the [`sensor_publisher` README](sensor_publisher/README.md) for the parameters and the timestamps. The nodes can also be started separately, as described below.

#### IMU

The ISM330DHCX driver uses I2C bus 0 and address `0x6A`. The sensor INT1 pin is wired to Raspberry Pi GPIO 23 (header pin 16). The node uses libgpiod 1.x to wait for rising-edge events on this line:

1. The IMU's gyroscope data-ready signal is routed to INT1.
2. The node waits for a GPIO 23 rising-edge event instead of polling the sensor continuously.
3. On an event, it reads the gyroscope and accelerometer output registers over I2C.
4. The kernel event timestamp is converted to the node's ROS clock and used as the message timestamp when the clock source can be mapped.
5. The node publishes the raw measurements on `/imu/data_raw`.

If multiple edge events are queued before the node reads the sensor, the driver processes the newest event because the sensor output registers contain the latest sample; older queued events are counted as missed. The event timestamp represents the GPIO data-ready edge, not a guaranteed measurement midpoint inside the sensor.

The node needs libgpiod:

```bash
sudo apt install libgpiod-dev gpiod
```

```bash
ros2 run sensor_publisher imu_node
```

Check the published data:

```bash
ros2 topic echo /imu/data_raw
ros2 topic hz /imu/data_raw
```

The user running the node must have permission to access `/dev/i2c-0`. If necessary, add the user to the `i2c` group and log out and back in.

#### Cameras

Start the dual-camera publisher:

```bash
ros2 launch sensor_publisher dual_camera.launch.py
```

Image topics:

- `/camera/fixed/image_raw`
- `/camera/rotating/image_raw`

Connect both cameras before starting the publisher.

### Vehicle sensors

The vehicle sensor packages are `driftmodel_msg` and `driftmodel`. Refer to their package-specific instructions before starting the sensor node.

## Notes

- Run the build and hardware checks on the Raspberry Pi after installing the dependencies.
- Camera and IMU topics are published independently. Their timestamps are converted to the Raspberry Pi ROS clock, but that alone does not prove physical camera-exposure and IMU-sampling alignment.
- In the tested 640 x 480 MJPG mode, both OV9281 cameras accepted 120 FPS although 30 FPS was requested; the publisher limits image publication to 30 FPS. See the sensor_publisher README for the available V4L2 modes and USB bandwidth considerations.
- The IMU publishes raw accelerometer and gyroscope values. It does not estimate orientation or apply bias calibration or a vehicle-frame axis transform.
- The IMU is configured for 208 Hz output. This is the configured ODR, not a measured ROS publication rate. In the existing bag CSV export, the observed header-stamp rate was 219.928 Hz; this discrepancy remains under investigation, and the export contains 82 fewer IMU timestamps than the bag metadata message count.
- Existing bag analysis found a median 9.167 ms same-index timestamp difference between the rotating and fixed cameras. This is not a physical exposure-time offset or proof of camera synchronization. Nearest-IMU-sample distances averaged 1.144 ms (fixed) and 1.147 ms (rotating); these are sample-proximity statistics, not camera-to-IMU synchronization errors. See the Foxy README for the full statistics and limitations.
