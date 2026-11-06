std_msgs/Header header

# operator input read from the STM32 board (voltage in V, 0 .. 3.3)
float32 steer

# One throttle value for both forward and brake.
# Below about 1.8 V the car drives forward, above about 1.8 V it brakes.
float32 throttle

# false: the operator drives, true: the laptop drives
bool mode_flag

# punishment switch of the operator
bool punishment_flag
