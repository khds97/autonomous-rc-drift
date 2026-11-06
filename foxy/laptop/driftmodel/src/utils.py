"""Constants and helpers shared by the laptop-side scripts."""

# Voltage range accepted by the STM32 command interface.
VOLT_MIN = 0.0
VOLT_MAX = 3.3

# Neutral command: steering centered, throttle at rest.
STEER_NEUTRAL = 1.525
THROTTLE_NEUTRAL = 1.815


def clamp(value, low=VOLT_MIN, high=VOLT_MAX):
    """Limit a value to [low, high]."""
    return max(low, min(high, float(value)))
