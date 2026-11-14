# IMU Publisher

ROS 2 Foxy package for an Adafruit ISM330DHCX 6-DoF IMU connected to the Raspberry Pi 4 over I2C.

## Connection

- I2C device: /dev/i2c-0
- I2C address: 0x6A
- WHO_AM_I: 0x6B

## Published topics

| Topic | Message type | Description |
|---|---|---|
| /imu/data_raw | sensor_msgs/msg/Imu | Unmodified angular velocity and linear acceleration |
| /imu/data_calibrated | sensor_msgs/msg/Imu | Gyroscope bias-corrected angular velocity and unchanged linear acceleration |

Both messages use imu_link as the frame ID. Angular velocity is published in rad/s and linear acceleration in m/s^2. Orientation is not estimated by this node.

## Sensor configuration

- Accelerometer: 104 Hz, +/-2 g
- Gyroscope: 104 Hz, +/-250 dps
- Block data update enabled
- Register address auto-increment enabled

The node checks the accelerometer and gyroscope data-ready flags before reading the output registers. The ROS timer polls at 100 Hz; the actual publish rate depends on the sensor data-ready status.

## Gyroscope bias calibration

At startup, the node collects 500 valid gyroscope samples while the IMU is stationary. It calculates the mean angular velocity on each axis and subtracts those values from subsequent gyroscope measurements.

During calibration:
- /imu/data_raw is published immediately.
- /imu/data_calibrated is not published until calibration is complete.
- Keep the vehicle and IMU completely still until the calibration-complete message appears.

The default sample count can be changed with a ROS parameter:

```bash
ros2 run imu_publisher imu_node --ros-args -p gyro_calibration_samples:=1000
```

At the configured 104 Hz sensor output rate, 500 samples take approximately 4.8 seconds and 1,000 samples take approximately 9.6 seconds.

This initial implementation calibrates the gyroscope only. Accelerometer bias calibration is not applied because a reliable accelerometer bias estimate requires measurements in multiple known orientations; simply subtracting gravity from one stationary pose can introduce an incorrect offset.

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

Check the topics:

```bash
ros2 topic echo /imu/data_raw
ros2 topic echo /imu/data_calibrated
ros2 topic hz /imu/data_calibrated
```

## Notes

- The message timestamp is assigned when the ROS message is published, not at the hardware measurement time.
- Sensor axes are published as reported by the IMU. No vehicle-frame rotation is applied.
- The orientation field is left unavailable.
- Camera-to-IMU calibration and time-offset calibration are not part of this package.
