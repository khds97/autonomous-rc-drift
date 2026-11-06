#!/usr/bin/env python3
"""Starts dagger.py when a PoseFlag message with flag == true arrives on /pose_flag.

Parameter:
  dagger_args  list of extra command line arguments for dagger.py, e.g. ['--epochs', '20']
"""

import os
import subprocess
import sys

import rclpy
from rclpy.node import Node

from driftmodel_msg.msg import PoseFlag


class PoseFlagNode(Node):

    def __init__(self):
        super().__init__('pose_flag')
        self.declare_parameter('dagger_args', [''])
        self.proc = None
        self.sub = self.create_subscription(PoseFlag, '/pose_flag', self.flag_cb, 1)

    def flag_cb(self, msg):
        self.get_logger().info('Receiving flag')

        if not msg.flag:
            self.get_logger().info('Proper condition not met, not starting dagger.py.')
            return

        if self.proc is not None and self.proc.poll() is None:
            self.get_logger().info('dagger.py is already running.')
            return

        script = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dagger.py')
        extra = [a for a in self.get_parameter('dagger_args').value if a]
        # Popen: the callback returns at once, so the node keeps spinning while dagger.py runs
        self.proc = subprocess.Popen([sys.executable, script] + extra)

    def destroy_node(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseFlagNode()
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
