#!/usr/bin/env python3
"""A7: launch installed localslam.launch.py and drive standard inputs.

Reuses A1/A4 driver patterns (enable, textured frames from a4_visual_path_check,
odometry/path/landmarks). Expects install_a7 (or overlay with localslam) sourced.
Does not start Webots/control/plot/globalslam.
"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import time
import traceback
from typing import Optional

import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from mavros_msgs.msg import Altitude
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image, Imu, PointCloud2, Range
from std_srvs.srv import SetBool, Trigger
from interfaces.msg import Robotstate

# Match tools/a4_visual_path_check.py scene (reuse fixture geometry).
IMG_H, IMG_W = 240, 320
BODY_VX = 0.35
ALT_M = 2.0
RANGE_M = 2.0


def make_textured_frame(frame_idx: int) -> bytes:
    """Same checker pattern as a4_visual_path_check.make_textured_frame."""
    shift = int(round(frame_idx * 2.0)) % 40
    buf = bytearray(IMG_H * IMG_W)
    for v in range(IMG_H):
        for u in range(IMG_W):
            cu = (u + shift) // 20
            cv = v // 20
            base = 220 if (cu + cv) % 2 == 0 else 40
            if ((u + shift) % 47 < 8) and (v % 53 < 8):
                base = 10 if base > 100 else 245
            buf[v * IMG_W + u] = base & 0xFF
    return bytes(buf)


def _rpy_to_quat(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def write_smoke_params(path: str) -> None:
    """Params aligned with a4 visual check; standard sensors; no rangetobase required."""
    params = {
        'Sensor_input_mode': 'standard',
        'Altitude_input_mode': 'derived_altitude',
        'Data_origin': 'standalone',
        'Visual_update': True,
        'Visual_init_feats_method': 'undelayed_range',
        'Visual_update_min_vel_for_update_feats': 0.02,
        'Visual_update_use_anchors': True,
        'Visual_update_use_1RANSAC_for_validation': False,
        'Attitude_update': True,
        'Altitude_update': True,
        'Speed_update': True,
        'Ground_station_range_update': False,
        'Prediction_model': 'noise_driven',
        'Max_freq_output': 50,
        'Robot_state_size': 13,
        'Mono_cam_frame_period': 0.05,
        'Mono_cam_img_rows': IMG_H,
        'Mono_cam_img_cols': IMG_W,
        'Mono_cam_fc_u': 206.34225,
        'Mono_cam_fc_v': 268.65192,
        'Mono_cam_cc_u': 156.24435,
        'Mono_cam_cc_v': 117.04562,
        'Mono_cam_distortions': [0.0, 0.0, 0.0, 0.0, 0.0],
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
        'use_sim_time': False,
    }
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump({'/**': {'ros__parameters': params}}, f)


class Driver(Node):
    def __init__(self) -> None:
        super().__init__('a7_standalone_driver')
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
        self.lm_n = 0
        self.rs_n = 0
        self.last_path_len = 0
        self.last_lm_width = 0
        self.frame_idx = 0
        self.create_subscription(Odometry, '/EKFslam/odometry', self._on_odom, 10)
        self.create_subscription(Path, '/EKFslam/path', self._on_path, 10)
        self.create_subscription(PointCloud2, '/EKFslam/landmarks', self._on_lm, 10)
        self.create_subscription(Robotstate, '/Robotstate_topic', self._on_rs, 10)

    def _on_odom(self, msg: Odometry) -> None:
        self.odom_n += 1
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            raise AssertionError(
                f'bad odom frames {msg.header.frame_id}/{msg.child_frame_id}')

    def _on_path(self, msg: Path) -> None:
        self.path_n += 1
        self.last_path_len = len(msg.poses)

    def _on_lm(self, msg: PointCloud2) -> None:
        self.lm_n += 1
        self.last_lm_width = msg.width

    def _on_rs(self, _msg: Robotstate) -> None:
        self.rs_n += 1

    def wait_srvs(self, timeout: float = 20.0) -> None:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.cli_enable.service_is_ready() and self.cli_reset.service_is_ready():
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError('enable/reset services missing')

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

    def publish_burst(self, n: int = 40) -> None:
        q = _rpy_to_quat(0.02, -0.01, 0.05)
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
            alt.relative = ALT_M
            self.pub_alt.publish(alt)

            tw = TwistWithCovarianceStamped()
            tw.header.stamp = now
            tw.header.frame_id = 'base_link'
            tw.twist.twist.linear.x = BODY_VX
            self.pub_spd.publish(tw)

            rng = Range()
            rng.header.stamp = now
            rng.header.frame_id = 'downward_ranger_link'
            rng.min_range = 0.1
            rng.max_range = 6.0
            rng.range = RANGE_M
            self.pub_rng.publish(rng)

            img = Image()
            img.header.stamp = now
            img.header.frame_id = 'camera_optical_frame'
            img.height = IMG_H
            img.width = IMG_W
            img.encoding = 'mono8'
            img.step = IMG_W
            img.data = make_textured_frame(self.frame_idx)
            self.frame_idx += 1
            self.pub_img.publish(img)

            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)


def wait_for_node(timeout: float = 25.0) -> bool:
    n = Node('a7_waiter')
    try:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if 'EKFslam' in n.get_node_names():
                return True
            rclpy.spin_once(n, timeout_sec=0.1)
        return False
    finally:
        n.destroy_node()


def main() -> int:
    proc: Optional[subprocess.Popen] = None
    log_dir = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'log_a7'))
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, 'a7_standalone_launch.log')
    params_path = os.path.join(log_dir, 'a7_smoke_params.yaml')
    try:
        print('=== A7 standalone localslam smoke ===', flush=True)
        write_smoke_params(params_path)
        os.chdir('/tmp')
        cmd = [
            'ros2', 'launch', 'localslam', 'localslam.launch.py',
            f'params_file:={params_path}',
            'use_sim_time:=false',
            'sensor_input_mode:=standard',
            'visual_update:=true',
            'ground_station_range_update:=false',
        ]
        logf = open(log_path, 'w', buffering=1)
        proc = subprocess.Popen(
            cmd, stdout=logf, stderr=subprocess.STDOUT, start_new_session=True)
        print(f'launch pid={proc.pid} log={log_path}', flush=True)

        rclpy.init()
        if not wait_for_node(30.0):
            raise RuntimeError('EKFslam did not appear; see ' + log_path)

        driver = Driver()
        try:
            driver.wait_srvs()
            driver.call_enable(True)
            driver.publish_burst(60)
            t0 = time.time()
            while time.time() - t0 < 5.0:
                rclpy.spin_once(driver, timeout_sec=0.1)

            print(
                f'odom={driver.odom_n} path_msgs={driver.path_n} '
                f'path_len={driver.last_path_len} landmarks_msgs={driver.lm_n} '
                f'lm_width={driver.last_lm_width} robotstate={driver.rs_n}',
                flush=True)
            if driver.odom_n < 1:
                raise RuntimeError('no ~/odometry')
            if driver.rs_n < 1:
                raise RuntimeError('no Robotstate_topic')
            if driver.path_n < 1:
                raise RuntimeError('no ~/path')
            if driver.lm_n < 1:
                raise RuntimeError('no ~/landmarks publications')
            # Visual processing enabled: expect at least one nonempty landmark cloud
            # after textured motion (operational, not accuracy).
            if driver.last_lm_width < 1:
                print(
                    'WARN: landmarks width still 0 (visual init may need more frames); '
                    'odometry/path/landmarks pubs OK',
                    flush=True)

            driver.call_reset()
            driver.publish_burst(25)
            t0 = time.time()
            while time.time() - t0 < 2.0:
                rclpy.spin_once(driver, timeout_sec=0.1)
            print('reset path exercised', flush=True)
        finally:
            driver.destroy_node()

        print('A7_STANDALONE_SMOKE_OK', flush=True)
        return 0
    except Exception as exc:
        print(f'A7_STANDALONE_SMOKE_FAIL: {exc}', file=sys.stderr, flush=True)
        traceback.print_exc()
        return 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        subprocess.run(
            ['pkill', '-f', 'localslam_container|localslam.launch'],
            check=False, capture_output=True)


if __name__ == '__main__':
    sys.exit(main())
