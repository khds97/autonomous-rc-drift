# Raspberry Pi 4

ROS 2 Foxy packages for the Raspberry Pi 4.

## Packages

| Package | Description |
|---|---|
| `driftmodel_msg` | Custom `Sensors` message definitions |
| `driftmodel` | Vehicle sensor data node |
| `camera_publisher` | Publishes images from the fixed and rotating cameras |
| `imu_publisher` | Publishes accelerometer and gyroscope data from the ISM330DHCX |

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

git clone --filter=blob:none --sparse -b foxy https://github.com/khds97/autonomous-rc-drift.git

cd ~/autonomous-rc-drift
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

### IMU

The ISM330DHCX driver currently uses I2C bus 0 and address `0x6A`.

```bash
ros2 run imu_publisher imu_node
```

Check the published data:

```bash
ros2 topic echo /imu/data_raw
ros2 topic hz /imu/data_raw
```

The user running the node must have permission to access `/dev/i2c-0`. If necessary, add the user to the `i2c` group and log out and back in.

### Cameras

Start the dual-camera publisher:

```bash
ros2 launch camera_publisher dual_camera.launch.py
```

Image topics:

- `/camera/fixed/image_raw`
- `/camera/rotating/image_raw`

Connect both cameras before starting the publisher.

### Vehicle sensors

The vehicle sensor packages are `driftmodel_msg` and `driftmodel`. Refer to their package-specific instructions before starting the sensor node.

## Notes

- Run the build and hardware checks on the Raspberry Pi after installing the dependencies.
- Camera and IMU topics are published independently; this setup does not provide hardware synchronization between the two cameras.
