# driftmodel_msg (Raspberry Pi)

Message definition of the vehicle sensor data. The Raspberry Pi package `driftmodel` publishes it on `/sensors`, and the laptop subscribes to it.

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | `stamp` is the time the sampling started (Pi clock, synchronized to the laptop with chrony), `frame_id` is `base_link` |
| `rpm` | `float32` | Motor speed from the 3-phase encoder [rpm], unsigned |
| `tire_pwm` | `int32` | Pulse width on the TIRE pin [us], about 1500 at neutral |
| `fl_speed`, `fr_speed`, `rl_speed`, `rr_speed` | `float32` | Wheel speeds [rpm], unsigned, 0 after `rpm_timeout_s` without a pulse |
| `current` | `float32` | ADS1115 channel 0 voltage [V] |

## Relation to `driftmodel`

`driftmodel` depends on this package (`find_package(driftmodel_msg)`), so this package has to be built first. `colcon` handles the order when both are in the same workspace.

The laptop has its own `driftmodel_msg` (`foxy/laptop/driftmodel_msg`) that contains the same `Sensors.msg` plus `CtrlVal`, `PoseFlag` and the `Flag` service. The two `Sensors.msg` files must stay identical. If they differ, the laptop cannot receive the messages. After changing one, copy the change to the other and rebuild both machines.

## Build

```bash
cd ~/foxy_ws
colcon build --packages-select driftmodel_msg
source install/setup.bash

ros2 interface show driftmodel_msg/msg/Sensors
```
