# IMU Publisher

ROS 2 Foxy package for an Adafruit ISM330DHCX 6-DoF IMU connected to the Raspberry Pi 4 over I2C. The node is interrupt driven: the sensor signals every new sample on its INT1 pin, and the node reads and publishes that sample.

## Connection

| Signal | Connection |
|---|---|
| I2C device | `/dev/i2c-0`, address `0x6A` (WHO_AM_I `0x6B`) |
| INT1 | GPIO 23 (header pin 16) on `gpiochip0` |

## Requirements

The GPIO edge events are read with libgpiod (1.x API):

```bash
sudo apt install libgpiod-dev gpiod
```

Check the INT1 wiring before starting the node. The sensor has to be running for edges to appear, so start the node first and look at the line from another terminal, or configure the sensor by hand and watch the pin with `gpiomon --rising-edge gpiochip0 23`.

## Published topics

| Topic | Message type | Description |
|---|---|---|
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | Unmodified angular velocity (rad/s) and linear acceleration (m/s^2) |

The frame ID is `imu_link`. Orientation is not estimated by this node (`orientation_covariance[0] = -1`).

## Sensor configuration

- Accelerometer: 208 Hz, +/-4 g
- Gyroscope: 208 Hz, +/-1000 dps
- Block data update and register address auto-increment enabled
- The sensor is software-reset at startup
- Gyroscope data-ready is routed to INT1 as a short pulse, one rising edge per sample. The accelerometer runs at the same output rate, so both are read after each edge.

If you change the output rate or the full-scale ranges, update the sensitivity constants in `src/imu_node.cpp` as well.

## Timestamps

The header stamp is the time of the INT1 edge reported by the kernel, converted to the node clock. If the edge time does not look like the current time, the node falls back to the time of reading. The node logs the publish rate and the number of missed samples every 5 seconds.

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `i2c_device` | `/dev/i2c-0` | I2C bus device |
| `frame_id` | `imu_link` | Frame ID of the published messages |
| `gpio_chip` | `gpiochip0` | GPIO chip the INT1 pin is connected to |
| `gpio_line` | `23` | Line offset (BCM number on a Raspberry Pi 4) |
| `event_timeout_ms` | `200` | Warn if no edge arrives for this long |

Example:

```bash
ros2 run imu_publisher imu_node --ros-args -p gpio_line:=23
```

## Build

```bash
cd ~/autonomous-rc-drift/foxy/RPi4
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select imu_publisher
source install/setup.bash
```

## Run

The user running the node needs access to `/dev/i2c-0` (the `i2c` group) and to `/dev/gpiochip0` (the `gpio` group). Log out and back in after changing groups.

```bash
ros2 run imu_publisher imu_node
```

Check the topic:

```bash
ros2 topic echo /imu/data_raw
ros2 topic hz /imu/data_raw
```

With the IMU lying still, the gyroscope values should be close to zero and the acceleration magnitude close to 9.8 m/s^2.

## Notes

- Sensor axes are published as reported by the IMU. No vehicle-frame rotation is applied.
- Gyroscope and accelerometer biases are not corrected in this node.
- Camera-to-IMU calibration and time-offset calibration are not part of this package.
