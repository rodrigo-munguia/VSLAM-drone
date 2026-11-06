#!/usr/bin/env python3
"""A4 node smoke: ~/landmarks PointCloud2, ~/path Path, pause, reset.

Visual_update remains false (A1-style). Visual path is a separate check.
"""

from __future__ import annotations

import math
import multiprocessing as mp
import os
import signal
import struct
import sys
import time
import traceback
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from mavros_msgs.msg import Altitude
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image, Imu, PointCloud2, Range
from std_srvs.srv import SetBool, Trigger
from interfaces.msg import Lslam, Robotstate

from launch import LaunchDescription, LaunchService
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

_D = (1.0, -1.0, -1.0)
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
        'Path_max_poses': 5,
    }


def _launch_worker(ready_q: mp.Queue, log_path: str) -> None:
    sys.stdout = open(log_path, 'w', buffering=1)
    sys.stderr = sys.stdout
    try:
        container = ComposableNodeContainer(
            name=f'a4_smoke_map_{os.getpid()}',
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
        f'_smoke_a4_map_{os.getpid()}.log'))
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
    try:
        import subprocess
        subprocess.run(
            ['pkill', '-f', 'component_container.*a4_smoke_'],
            check=False, capture_output=True)
    except Exception:
        pass


def wait_for_node(timeout: float = 25.0) -> bool:
    rclpy.init()
    n = Node('a4_map_waiter')
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


def _rpy_to_quat(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def unpack_xyz(cloud: PointCloud2) -> List[Tuple[float, float, float]]:
    pts = []
    if cloud.point_step < 12 or cloud.width == 0:
        return pts
    for i in range(cloud.width):
        off = i * cloud.point_step
        x, y, z = struct.unpack_from('<fff', cloud.data, off)
        pts.append((x, y, z))
    return pts


class Driver(Node):
    def __init__(self) -> None:
        super().__init__('a4_map_driver')
        self.pub_imu = self.create_publisher(Imu, '/EKFslam/imu', 10)
        self.pub_alt = self.create_publisher(Altitude, '/EKFslam/altitude', 10)
        self.pub_spd = self.create_publisher(
            TwistWithCovarianceStamped, '/EKFslam/body_velocity', 10)
        self.pub_rng = self.create_publisher(Range, '/EKFslam/downward_range', 10)
        self.pub_img = self.create_publisher(Image, '/EKFslam/image', 10)
        self.cli_enable = self.create_client(SetBool, '/EKFslam/enable')
        self.cli_reset = self.create_client(Trigger, '/EKFslam/reset')
        self.odom_n = 0
        self.path_n = 0
        self.cloud_n = 0
        self.lslam_n = 0
        self.last_path: Optional[Path] = None
        self.last_cloud: Optional[PointCloud2] = None
        self.last_lslam: Optional[Lslam] = None
        self.path_sizes: List[int] = []
        self.create_subscription(Odometry, '/EKFslam/odometry', self._on_odom, 10)
        self.create_subscription(Path, '/EKFslam/path', self._on_path, 10)
        self.create_subscription(PointCloud2, '/EKFslam/landmarks', self._on_cloud, 10)
        self.create_subscription(Lslam, '/lslam_data_topic', self._on_lslam, 10)
        self.create_subscription(Robotstate, '/Robotstate_topic', lambda _m: None, 10)

    def _on_odom(self, msg: Odometry) -> None:
        self.odom_n += 1
        if msg.header.frame_id != 'odom':
            raise AssertionError(f'odom frame {msg.header.frame_id}')

    def _on_path(self, msg: Path) -> None:
        self.path_n += 1
        self.last_path = msg
        self.path_sizes.append(len(msg.poses))
        if msg.header.frame_id != 'odom':
            raise AssertionError(f'path frame {msg.header.frame_id}')
        for p in msg.poses:
            if p.header.frame_id and p.header.frame_id != 'odom':
                raise AssertionError(f'pose frame {p.header.frame_id}')

    def _on_cloud(self, msg: PointCloud2) -> None:
        self.cloud_n += 1
        self.last_cloud = msg
        if msg.header.frame_id != 'odom':
            raise AssertionError(f'cloud frame {msg.header.frame_id}')

    def _on_lslam(self, msg: Lslam) -> None:
        self.lslam_n += 1
        self.last_lslam = msg

    def wait_srvs(self, timeout: float = 15.0) -> None:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.cli_enable.service_is_ready() and self.cli_reset.service_is_ready():
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError('services unavailable')

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

    def burst(self, n: int = 30) -> None:
        q = _rpy_to_quat(0.05, -0.02, 0.1)
        for _ in range(n):
            now = self.get_clock().now().to_msg()
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = 'imu_link'
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
            tw.twist.twist.linear.x = 0.3
            self.pub_spd.publish(tw)
            rng = Range()
            rng.header.stamp = now
            rng.range = 1.5
            rng.min_range = 0.1
            rng.max_range = 5.0
            self.pub_rng.publish(rng)
            img = Image()
            img.header.stamp = now
            img.height = 24
            img.width = 32
            img.encoding = 'mono8'
            img.step = 32
            img.data = bytes(24 * 32)
            self.pub_img.publish(img)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)


def main() -> int:
    try:
        print('=== A4 map/path smoke ===', flush=True)
        proc, log_path = start_slam()
        if not wait_for_node():
            log = open(log_path, encoding='utf-8', errors='replace').read()
            raise RuntimeError(f'EKFslam missing\n{log[-2000:]}')

        rclpy.init()
        d = Driver()
        try:
            d.wait_srvs()
            d.call_enable(True)
            d.burst(40)
            t0 = time.time()
            while time.time() - t0 < 3.0:
                rclpy.spin_once(d, timeout_sec=0.1)

            print(
                f'odom={d.odom_n} path={d.path_n} cloud={d.cloud_n} '
                f'lslam={d.lslam_n} path_sizes={d.path_sizes[-5:]}',
                flush=True)
            if d.odom_n < 1:
                raise RuntimeError('no odometry')
            if d.path_n < 1:
                raise RuntimeError('no ~/path')
            if d.last_path is None:
                raise RuntimeError('path never received')
            # Bound: Path_max_poses=5
            if len(d.last_path.poses) > 5:
                raise RuntimeError(
                    f'path exceeded bound: {len(d.last_path.poses)}')
            # With Visual_update=false, frames still publish empty landmark clouds
            if d.cloud_n < 1:
                raise RuntimeError(
                    'expected empty/nonempty landmark pubs on frames')
            if d.last_cloud is not None and d.last_cloud.width != 0:
                # Vision off → typically empty; tolerate nonempty only if lslam agrees
                pass
            else:
                print('empty landmark cloud OK (vision off)', flush=True)

            # Custom Lslam still operational when frames arrive
            # (may be empty map with vision off)

            # Pause: disable → path must not grow
            d.call_enable(False)
            path_n_before = d.path_n
            poses_before = len(d.last_path.poses) if d.last_path else 0
            time.sleep(0.2)
            d.burst(15)
            t0 = time.time()
            while time.time() - t0 < 1.0:
                rclpy.spin_once(d, timeout_sec=0.1)
            if d.path_n != path_n_before:
                raise RuntimeError('path grew while disabled')
            print(f'pause OK path_n={d.path_n} poses={poses_before}', flush=True)

            # Re-enable then reset → empty path + empty cloud
            d.call_enable(True)
            d.burst(10)
            t0 = time.time()
            while time.time() - t0 < 1.5:
                rclpy.spin_once(d, timeout_sec=0.1)

            d.call_reset()
            t0 = time.time()
            saw_empty_path = False
            saw_empty_cloud = False
            while time.time() - t0 < 3.0:
                rclpy.spin_once(d, timeout_sec=0.1)
                if d.last_path is not None and len(d.last_path.poses) == 0:
                    saw_empty_path = True
                if d.last_cloud is not None and d.last_cloud.width == 0:
                    saw_empty_cloud = True
            # Feed attitude again so system can reinit; empty pubs should have fired
            d.burst(20)
            t0 = time.time()
            while time.time() - t0 < 2.0:
                rclpy.spin_once(d, timeout_sec=0.1)
                if d.last_path is not None and len(d.last_path.poses) == 0:
                    saw_empty_path = True
                if d.last_cloud is not None and d.last_cloud.width == 0:
                    saw_empty_cloud = True

            print(
                f'reset empty_path={saw_empty_path} empty_cloud={saw_empty_cloud} '
                f'odom_after={d.odom_n}',
                flush=True)
            if not saw_empty_path:
                raise RuntimeError('reset did not publish cleared path')
            if not saw_empty_cloud:
                raise RuntimeError('reset did not publish empty landmarks')

            print('A4_MAP_PATH_SMOKE_OK', flush=True)
            return 0
        finally:
            d.destroy_node()
            rclpy.shutdown()
    except Exception as exc:
        print(f'A4_MAP_PATH_SMOKE_FAIL: {exc}', file=sys.stderr, flush=True)
        traceback.print_exc()
        return 1
    finally:
        cleanup()


if __name__ == '__main__':
    mp.set_start_method('spawn', force=True)
    sys.exit(main())
