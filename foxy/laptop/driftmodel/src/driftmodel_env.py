"""ROS 2 environment for the drift car.

Subscribes to the odometry, the vehicle sensors, the IMU and the operator input, and joins them with an
ApproximateTimeSynchronizer. Every synchronized set becomes one `Sample`: the 21-dim state and the operator
action that belong to the same time stamp.

API (gym style, without the gym dependency):
    sample = env.reset()
    sample, reward, done, info = env.step(action)     # action = [steer, throttle] in volts
"""

import threading
import time
from collections import namedtuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from driftmodel_msg.msg import Sensors, CtrlVal

from preprocess import FEATURE_NAMES
from usb_receiver import USBReceiver
from utils import clamp

try:
    from pynput import keyboard
except Exception:       # not installed, or no display available
    keyboard = None

# seq            : counter of the synchronized sets
# stamp          : time stamp of the odometry message [s]
# recv_time      : time.monotonic() when the set was received
# state          : float32 array, order of FEATURE_NAMES
# expert_action  : float32 array [steer, throttle] of the operator at the same time stamp
Sample = namedtuple('Sample', ['seq', 'stamp', 'recv_time', 'state', 'expert_action',
                               'mode_flag', 'punishment_flag'])


class DriftEnv(Node):

    def __init__(self, serial_port='/dev/ACMSTM32', use_usb=True, node_name='gym_ros_drift'):
        super().__init__(node_name)

        self.declare_parameter('odom_topic', '/loop_fusion_node/odometry_rect')
        self.declare_parameter('imu_topic', '/stereo_inertial_publisher/imu')
        self.declare_parameter('sensors_topic', 'sensors')
        self.declare_parameter('ctrl_val_topic', 'ctrl_val')
        self.declare_parameter('sync_queue_size', 100)
        self.declare_parameter('sync_slop', 0.1)
        self.declare_parameter('step_period', 0.1)      # minimum time between a command and the returned sample [s]
        self.declare_parameter('sample_timeout', 1.0)   # how long step() waits for a new sample [s]

        self._step_period = float(self.get_parameter('step_period').value)
        self._sample_timeout = float(self.get_parameter('sample_timeout').value)

        self._cond = threading.Condition()
        self._sample = None
        self._seq = 0

        # episode state used by the reward
        self._start_xy = (0.0, 0.0)
        self._loop_flag = 1
        self._last_mode = None

        self._stop_requested = threading.Event()
        self._executor = None
        self._spin_thread = None

        # sensor-data QoS (best effort) matches both reliable and best-effort publishers
        self.odom_sub = message_filters.Subscriber(
            self, Odometry, self.get_parameter('odom_topic').value, qos_profile_sensor_data)
        self.sensors_sub = message_filters.Subscriber(
            self, Sensors, self.get_parameter('sensors_topic').value, qos_profile_sensor_data)
        self.imu_sub = message_filters.Subscriber(
            self, Imu, self.get_parameter('imu_topic').value, qos_profile_sensor_data)
        self.ctrl_val_sub = message_filters.Subscriber(
            self, CtrlVal, self.get_parameter('ctrl_val_topic').value, qos_profile_sensor_data)

        self.tss = message_filters.ApproximateTimeSynchronizer(
            [self.odom_sub, self.sensors_sub, self.imu_sub, self.ctrl_val_sub],
            int(self.get_parameter('sync_queue_size').value),
            float(self.get_parameter('sync_slop').value))
        self.tss.registerCallback(self.callback)

        self.usb = USBReceiver(port=serial_port) if use_usb else None

        self.listener = None
        if keyboard is not None:
            try:
                self.listener = keyboard.Listener(on_press=self.on_press)
                self.listener.start()
            except Exception as exc:
                self.get_logger().warning('keyboard listener not available: {}'.format(exc))
        else:
            self.get_logger().warning('pynput is not available: press Ctrl+C to end an episode')

    # ------------------------------------------------------------------ ROS side
    def callback(self, odometry, sensors, imu, ctrl_val):
        p = odometry.pose.pose.position
        q = odometry.pose.pose.orientation
        v = odometry.twist.twist.linear

        values = {
            'px': p.x, 'py': p.y, 'pz': p.z,
            'qw': q.w, 'qx': q.x, 'qy': q.y, 'qz': q.z,
            'vx': v.x, 'vy': v.y, 'vz': v.z,
            'ax': imu.linear_acceleration.x,
            'ay': imu.linear_acceleration.y,
            'az': imu.linear_acceleration.z,
            'gx': imu.angular_velocity.x,
            'gy': imu.angular_velocity.y,
            'gz': imu.angular_velocity.z,
            'rpm': sensors.rpm,
            'tire_pwm': float(sensors.tire_pwm),
            'rl_speed': sensors.rl_speed,
            'rr_speed': sensors.rr_speed,
            'current': sensors.current,
        }
        state = np.array([values[name] for name in FEATURE_NAMES], dtype=np.float32)
        expert = np.array([ctrl_val.steer, ctrl_val.throttle], dtype=np.float32)
        stamp = odometry.header.stamp.sec + odometry.header.stamp.nanosec * 1e-9

        with self._cond:
            self._seq += 1
            self._sample = Sample(self._seq, stamp, time.monotonic(), state, expert,
                                  bool(ctrl_val.mode_flag), bool(ctrl_val.punishment_flag))
            self._cond.notify_all()

    def spin_in_background(self):
        """Spin the node in a separate thread so that the training loop can run in the main thread."""
        from rclpy.executors import SingleThreadedExecutor
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    # ------------------------------------------------------------------ keyboard
    def on_press(self, key):
        if key == keyboard.Key.esc:
            self._stop_requested.set()
            self.get_logger().info('you pressed esc')

    # ------------------------------------------------------------------ gym-style API
    def _wait_sample(self, newer_than, timeout):
        """Newest sample that was received at or after `newer_than` (time.monotonic()), or None on timeout."""
        with self._cond:
            ok = self._cond.wait_for(
                lambda: self._sample is not None and self._sample.recv_time >= newer_than,
                timeout=timeout)
            return self._sample if ok else None

    def reset(self, timeout=None):
        """Waits for a fresh sample and starts a new episode. Returns that sample."""
        self._stop_requested.clear()
        sample = self._wait_sample(time.monotonic(), timeout)
        if sample is None:
            raise TimeoutError('no synchronized sensor data received. Are odometry, IMU, '
                               '/sensors and /ctrl_val all being published?')
        self._start_xy = (float(sample.state[0]), float(sample.state[1]))
        self._loop_flag = 1
        self._last_mode = None
        return sample

    def step(self, action):
        """Sends the command and returns the first sample received at least `step_period` later.

        Returns (sample, reward, done, info). info['timeout'] is True when no new sample arrived in time;
        the previous sample is returned in that case.
        """
        steer = clamp(action[0])
        throttle = clamp(action[1])

        info = {}
        sent = time.monotonic()
        if self.usb is not None:
            try:
                self.usb.transmit_data(steer, throttle)
            except Exception as exc:
                info['usb_error'] = str(exc)
                self.get_logger().error('could not send the command: {}'.format(exc))

        sample = self._wait_sample(sent + self._step_period, self._step_period + self._sample_timeout)
        if sample is None:
            info['timeout'] = True
            self.get_logger().warning('no new sensor data for {:.1f} s'.format(self._sample_timeout))
            with self._cond:
                sample = self._sample
            if sample is None:
                raise RuntimeError('no sensor data has been received yet')
            return sample, 0.0, self._stop_requested.is_set(), info

        done = self._stop_requested.is_set()
        reward = self._compute_reward(sample, done)
        return sample, reward, done, info

    def send_neutral(self):
        """Sends the neutral steering / throttle command (used when an episode ends or the program stops)."""
        if self.usb is None:
            return
        try:
            self.usb.send_neutral()
        except Exception as exc:
            self.get_logger().error('could not send the neutral command: {}'.format(exc))

    def close(self):
        if self.listener is not None:
            try:
                self.listener.stop()
            except Exception:
                pass
        if self._executor is not None:
            self._executor.shutdown()
        if self.usb is not None:
            self.usb.close()

    # ------------------------------------------------------------------ reward
    def _compute_reward(self, sample, done):
        """Reward of one step. It is not used for training the policy.

        The value starts from 0 at every step, so a step that matches no rule gets 0.
        """
        px, py = float(sample.state[0]), float(sample.state[1])
        vx, vy = float(sample.state[7]), float(sample.state[8])
        rpm = float(sample.state[FEATURE_NAMES.index('rpm')])

        reward = 0.0
        dist_start = abs(self._start_xy[0] - px) + abs(self._start_xy[1] - py)

        if dist_start < 0.01 and not done and rpm == 0:
            reward = -0.5
            self.get_logger().debug('keep staying at the start, negative reward')
        elif abs(vx) < 0.005 and abs(vy) < 0.005:
            reward = -0.5
            self.get_logger().debug('velocity is 0, not moving, negative reward')
        elif (abs(px) + abs(py) > 0.0001) and rpm > 0:
            reward = 1.0
            if abs(px + py) > 2 and self._loop_flag == 1:
                self._loop_flag = 0

        if dist_start < 1 and self._loop_flag == 0:
            reward = 100.0
            self._loop_flag = 1
            self.get_logger().info('Congrats! You just made one loop!')

        if sample.mode_flag != self._last_mode:
            self.get_logger().info('autonomous steer mode' if sample.mode_flag else 'manual steer mode')
            self._last_mode = sample.mode_flag

        if sample.punishment_flag:
            self.get_logger().info('punishment!')
            reward = -10.0

        return reward
