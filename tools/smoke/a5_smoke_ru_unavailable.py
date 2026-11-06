#!/usr/bin/env python3
"""A5 bounded check: odometry feedback leaves r_u unavailable (NaN path).

Does not start the control mission stack. Verifies Local SLAM still publishes
odometry while documenting that pose covariance is present but is not a Pc
radius. Control-side NaN handling is covered by unit tests (test_go_home_ru).
"""

from __future__ import annotations

import math
import multiprocessing as mp
import os
import signal
import sys
import time
import traceback
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from mavros_msgs.msg import Altitude
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Range
from std_srvs.srv import SetBool
from interfaces.msg import Robotstate

from launch import LaunchDescription, LaunchService
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

CHILDREN: List[mp.Process] = []


def _params() -> dict:
    return {
        'Sensor_input_mode': 'standard',
        'Altitude_input_mode': 'derived_altitude',
        'Data_origin': 'webots',
        'Visual_update': False,
        'Attitude_update': True,
        'Altitude_update': True,
        'Speed_update': True,
        'Ground_station_range_update': False,
        'Prediction_model': 'noise_driven',
        'Max_freq_output': 50,
        'Robot_state_size': 13,
        'CL_Compute_Cp_from_start': True,
        'CL_Sigma_x': 0.2,
        'CL_Sigma_y': 0.2,
        'CL_min_vel_cl_uncertainty_update': 0.05,
        'Odometry_frame_id': 'odom',
        'Odometry_child_frame_id': 'base_link',
        'Stats': False,
    }


def _launch_worker(ready_q: mp.Queue, log_path: str) -> None:
    sys.stdout = open(log_path, 'w', buffering=1)
    sys.stderr = sys.stdout
    try:
        container = ComposableNodeContainer(
            name=f'a5_ru_{os.getpid()}',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='localslam',
                    plugin='localslam::EKFslam',
                    name='EKFslam',
                    parameters=[_params()],
                ),
            ],
            output='screen',
        )
        ls = LaunchService()
        ls.include_launch_description(LaunchDescription([container]))
        ready_q.put('starting')
        sys.exit(ls.run())
    except Exception:
        traceback.print_exc()
        ready_q.put('failed')
        sys.exit(1)


def start_slam() -> Tuple[mp.Process, str]:
    log_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'build_a1',
        f'_smoke_a5_ru_{os.getpid()}.log'))
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    ready_q: mp.Queue = mp.Queue()
    proc = mp.Process(target=_launch_worker, args=(ready_q, log_path), daemon=True)
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
    for p in list(CHILDREN):
        p.join(timeout=2.0)
        if p.is_alive():
            try:
                os.kill(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    CHILDREN.clear()
    try:
        import subprocess
        subprocess.run(
            ['pkill', '-f', 'component_container.*a5_ru_'],
            check=False, capture_output=True)
    except Exception:
        pass


def wait_for_node(timeout: float = 25.0) -> bool:
    rclpy.init()
    n = Node('a5_waiter')
    try:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if 'EKFslam' in n.get_node_names():
                return True
            rclpy.spin_once(n, timeout_sec=0.1)
        return False
    finally:
        n.destroy_node()
        rclpy.shutdown()


def _rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class Driver(Node):
    def __init__(self) -> None:
        super().__init__('a5_ru_driver')
        self.pub_imu = self.create_publisher(Imu, '/EKFslam/imu', 10)
        self.pub_alt = self.create_publisher(Altitude, '/EKFslam/altitude', 10)
        self.pub_spd = self.create_publisher(
            TwistWithCovarianceStamped, '/EKFslam/body_velocity', 10)
        self.pub_rng = self.create_publisher(Range, '/EKFslam/downward_range', 10)
        self.cli_enable = self.create_client(SetBool, '/EKFslam/enable')
        self.odom_n = 0
        self.robot_n = 0
        self.last_odom = None
        self.last_ru = None
        self.create_subscription(Odometry, '/EKFslam/odometry', self._on_odom, 10)
        self.create_subscription(Robotstate, '/Robotstate_topic', self._on_robot, 10)

    def _on_odom(self, msg: Odometry) -> None:
        self.odom_n += 1
        self.last_odom = msg

    def _on_robot(self, msg: Robotstate) -> None:
        self.robot_n += 1
        self.last_ru = float(msg.radius_uncertainty)

    def wait_srvs(self) -> None:
        t0 = time.time()
        while time.time() - t0 < 15.0:
            if self.cli_enable.service_is_ready():
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError('enable unavailable')

    def enable(self) -> None:
        req = SetBool.Request()
        req.data = True
        fut = self.cli_enable.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

    def burst(self, n: int = 40) -> None:
        q = _rpy_to_quat(0.05, -0.02, 0.1)
        for _ in range(n):
            now = self.get_clock().now().to_msg()
            imu = Imu()
            imu.header.stamp = now
            imu.orientation.x, imu.orientation.y = q[0], q[1]
            imu.orientation.z, imu.orientation.w = q[2], q[3]
            imu.linear_acceleration.z = 9.81
            self.pub_imu.publish(imu)
            alt = Altitude()
            alt.header.stamp = now
            alt.relative = 2.0
            self.pub_alt.publish(alt)
            tw = TwistWithCovarianceStamped()
            tw.header.stamp = now
            tw.header.frame_id = 'base_link'
            tw.twist.twist.linear.x = 0.4  # above CL_min_vel
            self.pub_spd.publish(tw)
            rng = Range()
            rng.header.stamp = now
            rng.range = 1.5
            rng.min_range = 0.1
            rng.max_range = 5.0
            self.pub_rng.publish(rng)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)


def main() -> int:
    try:
        print('=== A5 r_u / odometry insufficiency smoke ===', flush=True)
        start_slam()
        if not wait_for_node():
            raise RuntimeError('EKFslam missing')
        rclpy.init()
        d = Driver()
        try:
            d.wait_srvs()
            d.enable()
            d.burst(50)
            t0 = time.time()
            while time.time() - t0 < 2.0:
                rclpy.spin_once(d, timeout_sec=0.1)
            print(
                f'odom={d.odom_n} robot={d.robot_n} last_ru={d.last_ru}',
                flush=True)
            if d.odom_n < 1 or d.last_odom is None:
                raise RuntimeError('no odometry')
            # Pose covariance exists on Odometry but is not Pc.
            cov = list(d.last_odom.pose.covariance)
            if not any(abs(c) > 0 for c in cov):
                print('note: pose cov may be near-zero early; still not Pc', flush=True)
            if d.last_ru is None:
                raise RuntimeError('no Robotstate radius_uncertainty samples')
            # With CL_Compute_Cp_from_start, legacy Robotstate may grow r_u;
            # Odometry still has no field for it — control must use legacy or
            # an approved CL-radius topic for go_home explore.
            print(
                f'Robotstate.radius_uncertainty={d.last_ru} '
                f'(Pc-based; not carried on Odometry)',
                flush=True)
            print(
                'A5: odometry insufficient for r_u; control NaN path remains '
                '(unit-tested). A5_RU_SMOKE_OK',
                flush=True)
            return 0
        finally:
            d.destroy_node()
            rclpy.shutdown()
    except Exception as exc:
        print(f'A5_RU_SMOKE_FAIL: {exc}', file=sys.stderr, flush=True)
        traceback.print_exc()
        return 1
    finally:
        cleanup()


if __name__ == '__main__':
    mp.set_start_method('spawn', force=True)
    sys.exit(main())
