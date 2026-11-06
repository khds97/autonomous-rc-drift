Sensor node of the Raspberry Pi 4. It reads the GPIO sensors through pigpio and the current sensor through an ADS1115, and publishes `sensors` (`driftmodel_msg/msg/Sensors`).

Requires the pigpio daemon: `sudo pigpiod`.
