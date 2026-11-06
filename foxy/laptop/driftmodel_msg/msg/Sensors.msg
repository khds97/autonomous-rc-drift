std_msgs/Header header

# motor speed from the 3-phase encoder [rpm], unsigned
float32 rpm

# pulse width measured on the TIRE pin [us] (about 1500 at neutral for a standard RC signal)
int32 tire_pwm

# wheel speeds [rpm], unsigned. One hall sensor per wheel, so the direction of rotation is not measured.
# Computed from the time between two magnet passes; 0 when no pulse arrived for rpm_timeout_s.
float32 fl_speed
float32 fr_speed
float32 rl_speed
float32 rr_speed

# ADS1115 channel 0 voltage [V]. Convert to current with the sensitivity of the current sensor.
float32 current
