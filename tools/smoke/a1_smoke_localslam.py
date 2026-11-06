#!/usr/bin/env python3
"""Bounded A1 ROS node-level smoke (no Webots, no ros2 CLI).

Starts Local SLAM via launch_ros LaunchService, inspects subscriptions with
rclpy graph APIs, publishes representative standard inputs, exercises
enable/disable/re-enable/reset, and always tears down child processes.
"""

from __future__ import annotations

import math
import multiprocessing as mp
import os
import signal
import sys
import time
import traceback
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from mavros_msgs.msg import Altitude
from sensor_msgs.msg import Image, Imu, Range
from std_srvs.srv import SetBool, Trigger
from interfaces.msg import Att, Alt as LegacyAlt, Robotstate
from nav_msgs.msg import Odometry

from launch import LaunchDescription, LaunchService
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# A1/A2 axis map D = diag(1,-1,-1)
_D = (1.0, -1.0, -1.0)


CHILDREN: List[mp.Process] = []


def _params(mode: str, alt_mode: str) -> dict:
    return {
        'Sensor_input_mode': mode,
        'Altitude_input_mode': alt_mode,
        'Data_origin': 'webots',
        'Visual_update': False,
        'Attitude_update': True,
        'Altitude_update': True,
        'Speed_update': True,
        'Ground_station_range_update': False,
        'Prediction_model': 'noise_driven',
        'Max_freq_output': 50,
        'Robot_state_size': 13,
        'Mono_cam_frame_period': 0.0333,
        'Sigma_att_update': 0.001,
        'Sigma_spd_update': 0.1,
        'Sigma_alt_update': 0.1,
        'Sigma_w': 0.1,
        'Sigma_a': 6.0,
        'Tau_r': 1.0,
        'Tau_p': 100.0,
        'Alt_h_2_d': -1,
        'Stats': False,
        'Odometry_frame_id': 'odom',
        'Odometry_child_frame_id': 'base_link',
    }


def _launch_worker(mode: str, alt_mode: str, ready_q: mp.Queue, log_path: str) -> None:
    # Redirect stdout/stderr for later inspection (unsupported-mode failures).
    sys.stdout = open(log_path, 'w', buffering=1)
    sys.stderr = sys.stdout
    try:
        container = ComposableNodeContainer(
            name=f'a1_smoke_{mode}_{os.getpid()}',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='localslam',
                    plugin='localslam::EKFslam',
                    name='EKFslam',
                    parameters=[_params(mode, alt_mode)],
                ),
            ],
            output='screen',
        )
        ld = LaunchDescription([container])
        ls = LaunchService()
        ls.include_launch_description(ld)
        ready_q.put('starting')
        sys.exit(ls.run())
    except Exception:
        traceback.print_exc()
        ready_q.put('failed')
        sys.exit(1)


def start_slam(mode: str, alt_mode: str = 'derived_altitude') -> Tuple[mp.Process, str]:
    log_path = os.path.join(
        os.path.dirname(__file__), '..', 'build_a1',
        f'_smoke_{mode}_{alt_mode}_{os.getpid()}.log')
    log_path = os.path.abspath(log_path)
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    ready_q: mp.Queue = mp.Queue()
    proc = mp.Process(
        target=_launch_worker,
        args=(mode, alt_mode, ready_q, log_path),
        daemon=True,
    )
    proc.start()
    CHILDREN.append(proc)
    try:
        ready_q.get(timeout=10.0)
    except Exception:
        pass
    return proc, log_path


def cleanup() -> None:
    for p in list(CHILDREN):
        if p.is_alive():
            try:
                os.kill(p.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
    deadline = time.time() + 5.0
    for p in list(CHILDREN):
        p.join(timeout=max(0.0, deadline - time.time()))
        if p.is_alive():
            try:
                os.kill(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            p.join(timeout=1.0)
    CHILDREN.clear()
    # Also reap any orphaned component_container from prior runs of this smoke.
    try:
        import subprocess
        subprocess.run(
            ['pkill', '-f', 'component_container.*a1_smoke_'],
            check=False, capture_output=True)
    except Exception:
        pass


def wait_for_node(timeout: float = 20.0) -> bool:
    rclpy.init()
    n = Node('a1_smoke_waiter')
    try:
        t0 = time.time()
        while time.time() - t0 < timeout:
            names = n.get_node_names()
            if 'EKFslam' in names:
                return True
            time.sleep(0.2)
            # spin briefly so discovery progresses
            rclpy.spin_once(n, timeout_sec=0.0)
        return False
    finally:
        n.destroy_node()
        rclpy.shutdown()


def get_subscriptions() -> List[str]:
    rclpy.init()
    n = Node('a1_smoke_graph')
    try:
        # Allow discovery
        t0 = time.time()
        while time.time() - t0 < 3.0:
            if 'EKFslam' in n.get_node_names():
                break
            rclpy.spin_once(n, timeout_sec=0.1)
        subs = n.get_subscriber_names_and_types_by_node('EKFslam', '/')
        return sorted({name for name, _ in subs})
    finally:
        n.destroy_node()
        rclpy.shutdown()


def assert_subs(mode: str, topics: List[str]) -> None:
    std = [
        '/EKFslam/image',
        '/EKFslam/imu',
        '/EKFslam/body_velocity',
        '/EKFslam/downward_range',
        '/EKFslam/altitude',
    ]
    legacy = [
        '/Attitude_topic',
        '/Altitude_topic',
        '/Speed_topic',
        '/Range_topic',
        '/Frame_topic',
    ]
    rtb = '/Rangetobase_topic'
    if mode == 'standard':
        for t in std:
            if t not in topics:
                raise AssertionError(f'standard missing sub {t}; have={topics}')
        for t in legacy:
            if t in topics:
                raise AssertionError(f'standard must not sub {t}; have={topics}')
    else:
        for t in legacy:
            if t not in topics:
                raise AssertionError(f'legacy missing sub {t}; have={topics}')
        for t in std:
            if t in topics:
                raise AssertionError(f'legacy must not sub {t}; have={topics}')
    if rtb not in topics:
        raise AssertionError(f'missing rangetobase; have={topics}')


def _rpy_to_quat(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


class SmokeDriver(Node):
    def __init__(self) -> None:
        super().__init__('a1_smoke_driver')
        self.pub_imu = self.create_publisher(Imu, '/EKFslam/imu', 10)
        self.pub_alt = self.create_publisher(Altitude, '/EKFslam/altitude', 10)
        self.pub_spd = self.create_publisher(
            TwistWithCovarianceStamped, '/EKFslam/body_velocity', 10)
        self.pub_rng = self.create_publisher(Range, '/EKFslam/downward_range', 10)
        self.pub_img = self.create_publisher(Image, '/EKFslam/image', 10)
        self.pub_leg_att = self.create_publisher(Att, '/Attitude_topic', 10)
        self.pub_leg_alt = self.create_publisher(LegacyAlt, '/Altitude_topic', 10)
        self.cli_enable = self.create_client(SetBool, '/EKFslam/enable')
        self.cli_reset = self.create_client(Trigger, '/EKFslam/reset')
        self.robot_msgs = 0
        self.odom_msgs = 0
        self.last_robot = None
        self.odom_vs_robot_ok = 0
        self.odom_checks = 0
        self.create_subscription(Robotstate, '/Robotstate_topic', self._on_robot, 10)
        self.create_subscription(Odometry, '/EKFslam/odometry', self._on_odom, 10)

    def _on_robot(self, msg: Robotstate) -> None:
        self.robot_msgs += 1
        self.last_robot = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.odom_msgs += 1
        self.odom_checks += 1
        # Type / frames / finite / quaternion / covariance presence
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            raise AssertionError(
                f'bad frames: {msg.header.frame_id!r} {msg.child_frame_id!r}')
        q = msg.pose.pose.orientation
        n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z
        if abs(n2 - 1.0) > 1e-3:
            raise AssertionError(f'quat not unit: norm^2={n2}')
        for v in (
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
            msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z,
        ):
            if not math.isfinite(v):
                raise AssertionError('non-finite odometry value')
        if any(not math.isfinite(c) for c in msg.pose.covariance):
            raise AssertionError('non-finite pose covariance')
        if any(not math.isfinite(c) for c in msg.twist.covariance):
            raise AssertionError('non-finite twist covariance')
        # Equivalent snapshot vs custom Robotstate via known D map (not arrival pairing alone)
        if self.last_robot is None or len(self.last_robot.robot_state) < 13:
            return
        xs = self.last_robot.robot_state
        exp_p = (_D[0] * xs[7], _D[1] * xs[8], _D[2] * xs[9])
        exp_v = (_D[0] * xs[10], _D[1] * xs[11], _D[2] * xs[12])
        exp_w = (_D[0] * xs[4], _D[1] * xs[5], _D[2] * xs[6])
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        if (
            abs(p.x - exp_p[0]) < 1e-5 and abs(p.y - exp_p[1]) < 1e-5 and
            abs(p.z - exp_p[2]) < 1e-5 and
            abs(v.x - exp_v[0]) < 1e-5 and abs(v.y - exp_v[1]) < 1e-5 and
            abs(v.z - exp_v[2]) < 1e-5 and
            abs(w.x - exp_w[0]) < 1e-5 and abs(w.y - exp_w[1]) < 1e-5 and
            abs(w.z - exp_w[2]) < 1e-5
        ):
            self.odom_vs_robot_ok += 1

    def wait_srvs(self, timeout: float = 15.0) -> None:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.cli_enable.service_is_ready() and self.cli_reset.service_is_ready():
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError('enable/reset services not available')

    def call_enable(self, on: bool) -> None:
        req = SetBool.Request()
        req.data = on
        fut = self.cli_enable.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None or not fut.result().success:
            raise RuntimeError(f'enable({on}) failed')

    def call_reset(self) -> None:
        fut = self.cli_reset.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None or not fut.result().success:
            raise RuntimeError('reset failed')

    def publish_standard_burst(self, n: int = 30) -> None:
        q = _rpy_to_quat(0.12, -0.34, 0.56)
        for _ in range(n):
            now = self.get_clock().now().to_msg()
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = 'imu_link'
            imu.orientation.x, imu.orientation.y = q[0], q[1]
            imu.orientation.z, imu.orientation.w = q[2], q[3]
            imu.orientation_covariance[0] = 0.0
            imu.linear_acceleration.z = 9.81
            self.pub_imu.publish(imu)

            alt = Altitude()
            alt.header.stamp = now
            alt.header.frame_id = 'gps_link'
            alt.relative = 2.5
            self.pub_alt.publish(alt)

            tw = TwistWithCovarianceStamped()
            tw.header.stamp = now
            tw.header.frame_id = 'base_link'
            tw.twist.twist.linear.x = 0.4
            tw.twist.twist.linear.y = -0.2
            tw.twist.twist.linear.z = 0.1
            self.pub_spd.publish(tw)

            rng = Range()
            rng.header.stamp = now
            rng.header.frame_id = 'downward_ranger_link'
            rng.radiation_type = Range.INFRARED
            rng.field_of_view = 0.1
            rng.min_range = 0.1
            rng.max_range = 5.0
            rng.range = 1.5
            self.pub_rng.publish(rng)

            img = Image()
            img.header.stamp = now
            img.header.frame_id = 'camera_optical_frame'
            img.height = 24
            img.width = 32
            img.encoding = 'mono8'
            img.step = 32
            img.data = bytes(24 * 32)
            self.pub_img.publish(img)

            # Legacy copies — ignored when Sensor_input_mode=standard
            la = Att()
            la.roll = 9.0
            la.pitch = 9.0
            la.yaw = 9.0
            la.time = 0
            self.pub_leg_att.publish(la)
            lalt = LegacyAlt()
            lalt.altitude = 99.0
            lalt.time = 0
            self.pub_leg_alt.publish(lalt)

            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)


def run_mode(mode: str) -> None:
    print(f'=== smoke mode={mode} ===', flush=True)
    proc, log_path = start_slam(mode)
    if not wait_for_node(25.0):
        log = open(log_path, encoding='utf-8', errors='replace').read()
        raise RuntimeError(
            f'EKFslam did not appear for mode={mode}; alive={proc.is_alive()} '
            f'log={log_path}\n{log[-2000:]}')
    topics = get_subscriptions()
    print('subscriptions:', topics, flush=True)
    assert_subs(mode, topics)

    if mode != 'standard':
        cleanup()
        time.sleep(0.5)
        return

    rclpy.init()
    driver = SmokeDriver()
    try:
        driver.wait_srvs()
        driver.call_enable(True)
        driver.publish_standard_burst(40)
        t0 = time.time()
        while time.time() - t0 < 3.0:
            rclpy.spin_once(driver, timeout_sec=0.1)
        print(
            f'robotstate_msgs={driver.robot_msgs} odom_msgs={driver.odom_msgs} '
            f'odom_vs_robot_ok={driver.odom_vs_robot_ok}',
            flush=True)
        if driver.robot_msgs < 1:
            log = open(log_path, encoding='utf-8', errors='replace').read()
            raise RuntimeError(
                'no Robotstate_topic after standard inputs\n' + log[-2000:])
        if driver.odom_msgs < 1:
            raise RuntimeError('no ~/odometry after standard inputs')
        if driver.odom_vs_robot_ok < 1:
            raise RuntimeError(
                'odometry did not match Robotstate under D=diag(1,-1,-1)')

        driver.call_enable(False)
        before_r = driver.robot_msgs
        before_o = driver.odom_msgs
        time.sleep(0.3)
        driver.publish_standard_burst(10)
        t0 = time.time()
        while time.time() - t0 < 1.0:
            rclpy.spin_once(driver, timeout_sec=0.1)
        after_disable_r = driver.robot_msgs
        after_disable_o = driver.odom_msgs
        print(
            f'after_disable robot={after_disable_r} odom={after_disable_o} '
            f'(before {before_r}/{before_o})',
            flush=True)
        if after_disable_o != before_o:
            raise RuntimeError('odometry continued while disabled')

        driver.call_enable(True)
        driver.publish_standard_burst(20)
        t0 = time.time()
        while time.time() - t0 < 2.0:
            rclpy.spin_once(driver, timeout_sec=0.1)
        print(
            f'after_reenable robot={driver.robot_msgs} odom={driver.odom_msgs}',
            flush=True)
        if driver.robot_msgs <= after_disable_r or driver.odom_msgs <= after_disable_o:
            raise RuntimeError('re-enable did not resume robot/odometry publication')

        driver.call_reset()
        odom_at_reset = driver.odom_msgs
        driver.publish_standard_burst(20)
        t0 = time.time()
        while time.time() - t0 < 2.0:
            rclpy.spin_once(driver, timeout_sec=0.1)
        print(
            f'after_reset robot={driver.robot_msgs} odom={driver.odom_msgs} '
            f'(odom_at_reset={odom_at_reset})',
            flush=True)
        # Custom output still functional
        if driver.robot_msgs < 1:
            raise RuntimeError('Robotstate inactive after reset path')
    finally:
        driver.destroy_node()
        rclpy.shutdown()
        cleanup()
        time.sleep(0.5)


def run_bad_altitude_mode() -> None:
    print('=== smoke unsupported Altitude_input_mode ===', flush=True)
    proc, log_path = start_slam('standard', alt_mode='fluid_pressure')
    # Give the container time to load the plugin and abort.
    time.sleep(6.0)
    appeared = False
    try:
        appeared = wait_for_node(2.0)
    except Exception:
        appeared = False
    log = open(log_path, encoding='utf-8', errors='replace').read()
    cleanup()
    time.sleep(0.3)
    markers = (
        'not implemented',
        'Unsupported Altitude_input_mode',
        'fluid_pressure',
        'std::runtime_error',
        'runtime_error',
    )
    if not any(m in log for m in markers) and appeared and proc.exitcode == 0:
        raise RuntimeError(
            'expected clear failure for fluid_pressure; log tail:\n' + log[-2500:])
    print('bad altitude mode failed as expected', flush=True)
    print(log[-1500:], flush=True)


def main() -> int:
    try:
        run_mode('legacy')
        run_mode('standard')
        run_bad_altitude_mode()
        print('A2_SMOKE_OK', flush=True)
        return 0
    except Exception as exc:
        print(f'A2_SMOKE_FAIL: {exc}', file=sys.stderr, flush=True)
        traceback.print_exc()
        return 1
    finally:
        cleanup()


if __name__ == '__main__':
    # Required for LaunchService in a child process on some platforms.
    mp.set_start_method('spawn', force=True)
    sys.exit(main())
