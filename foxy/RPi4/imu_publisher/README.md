# IMU Publisher

ROS 2 Foxy package for an Adafruit ISM330DHCX 6-DoF IMU connected to the Raspberry Pi 4 over I2C.

## Connection

- I2C device: /dev/i2c-0
- I2C address: 0x6A
- WHO_AM_I: 0x6B

## Published topic

| Topic | Message type | Description |
|---|---|---|
| /imu/data_raw | sensor_msgs/msg/Imu | Angular velocity and linear acceleration |

The message uses imu_link as its frame ID. Angular velocity is published in rad/s and linear acceleration in m/s^2. Orientation is not estimated by this node.

## Sensor configuration

- Accelerometer: 104 Hz, +/-2 g
- Gyroscope: 104 Hz, +/-250 dps
- Block data update enabled
- Register address auto-increment enabled

The node checks the accelerometer and gyroscope data-ready flags before reading the output registers. The ROS timer polls at 100 Hz; the actual publish rate depends on the sensor data-ready status.

## Build

```bash
cd ~/foxy_ws
colcon build --packages-select imu_publisher
source install/setup.bash
```

## Run

Make sure the user running the node has permission to access the I2C device. On Ubuntu, this can usually be granted by adding the user to the i2c group and logging in again.

```bash
ros2 run imu_publisher imu_node
```

Check the topic:

```bash
ros2 topic echo /imu/data_raw
ros2 topic hz /imu/data_raw
```

## Notes

- The message timestamp is assigned when the ROS message is published, not at the hardware measurement time.
- Sensor axes are published as reported by the IMU. No vehicle-frame rotation or bias calibration is applied.
- The orientation field is left unavailable.
- Camera-to-IMU calibration and time-offset calibration are not part of this package.
