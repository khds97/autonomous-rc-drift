#!/usr/bin/env python3
"""Reads the operator input from the STM32 board and publishes it on `ctrl_val`.

Parameters:
  serial_port      serial device of the STM32 board (default /dev/ACMSTM32)
  baudrate         (default 115200)
  rate_hz          publish rate (default 100.0)
  stale_timeout_s  stop publishing when no valid line arrived for this long, so that stale data
                   cannot be mistaken for fresh data. 0 disables the check. (default 1.0)
"""

import os
import sys

# allow running from the source tree: usb_receiver.py is one directory above this file
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import rclpy
from rclpy.node import Node

from driftmodel_msg.msg import CtrlVal
from usb_receiver import USBReceiver


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        self.declare_parameter('serial_port', '/dev/ACMSTM32')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('stale_timeout_s', 1.0)

        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        rate_hz = float(self.get_parameter('rate_hz').value)
        self._stale_timeout = float(self.get_parameter('stale_timeout_s').value)

        self.usb = USBReceiver(port=port, baudrate=baudrate)
        self.pub = self.create_publisher(CtrlVal, 'ctrl_val', 100)
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)
        self._warned_stale = False

        self.get_logger().info('reading the STM32 board on {}'.format(port))

    def on_timer(self):
        self.usb.receive_data()

        if not self.usb.has_data:
            return

        if self._stale_timeout > 0.0:
            import time
            age = time.monotonic() - self.usb.last_rx_time
            if age > self._stale_timeout:
                if not self._warned_stale:
                    self.get_logger().warning(
                        'no valid data from the STM32 board for {:.1f} s, not publishing'.format(age))
                    self._warned_stale = True
                return
            self._warned_stale = False

        msg = CtrlVal()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # Publish the operator input in both modes, so the expert label is also
        # available while the laptop is driving.
        msg.steer = float(self.usb.steer)
        msg.throttle = float(self.usb.throttle)
        msg.mode_flag = bool(self.usb.mode_flag)
        msg.punishment_flag = bool(self.usb.punishment_flag)
        self.pub.publish(msg)

    def destroy_node(self):
        self.usb.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
