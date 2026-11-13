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

Make sure ROS 2 Foxy is installed and the ROS environment is available before building.

## Clone

Clone the `foxy` branch:

```bash
cd ~
git clone -b foxy https://github.com/khds97/autonomous-rc-drift.git
```

## Install dependencies

Source ROS 2 and install the dependencies declared by the packages:

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

Build all packages in this directory:

```bash
cd ~/autonomous-rc-drift/foxy/RPi4
source /opt/ros/foxy/setup.bash

colcon build --symlink-install
```

After the build, source the workspace:

```bash
source install/setup.bash
```

For new terminals, source both environments:

```bash
source /opt/ros/foxy/setup.bash
source ~/autonomous-rc-drift/foxy/RPi4/install/setup.bash
```

## Run

### IMU

The ISM330DHCX is connected through I2C. The current driver uses I2C bus 0 and address `0x6A`.

```bash
ros2 run imu_publisher imu_node
```

Check the published data:

```bash
ros2 topic echo /imu/data_raw
ros2 topic hz /imu/data_raw
```

### Cameras

Start the dual-camera publisher:

```bash
ros2 launch camera_publisher dual_camera.launch.py
```

The image topics are:

- `/camera/fixed/image_raw`
- `/camera/rotating/image_raw`

### Vehicle sensors

The vehicle sensor packages are `driftmodel_msg` and `driftmodel`. Refer to the package-specific instructions before starting the sensor node.

## Notes

- Connect the cameras and IMU before launching their publisher nodes.
- The user running the IMU node must have permission to access `/dev/i2c-0`. Add the user to the `i2c` group if necessary, then log out and back in.
- Build and runtime behavior should be checked on the Raspberry Pi after cloning.
