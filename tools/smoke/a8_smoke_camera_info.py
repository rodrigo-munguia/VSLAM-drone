#!/usr/bin/env python3
"""A8: compare YAML vs equivalent CameraInfo calibration on the textured fixture.

Also exercises missing/invalid/changed calibration behavior via CameraInfo mode.
Expects install_a8 (or overlay with localslam) sourced. No Webots.
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
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, Range
from std_srvs.srv import SetBool, Trigger
from interfaces.msg import Robotstate

IMG_H, IMG_W = 240, 320
BODY_VX = 0.35
ALT_M = 2.0
RANGE_M = 2.0
FC_U, FC_V = 206.34225, 268.65192
CC_U, CC_V = 156.24435, 117.04562
OPTICAL_FRAME = 'camera_optical_frame'


def make_textured_frame(frame_idx: int) -> bytes:
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


def base_params(calibration_source: str) -> dict:
    return {
        'Sensor_input_mode': 'standard',
        'Altitude_input_mode': 'derived_altitude',
        'Data_origin': 'standalone',
        'Calibration_source': calibration_source,
        'Camera_info_image_path': 'raw',
        'Camera_info_waiting_image_bound': 2,
        'Camera_info_expected_frame_id': OPTICAL_FRAME,
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
        'Mono_cam_fc_u': FC_U,
        'Mono_cam_fc_v': FC_V,
        'Mono_cam_cc_u': CC_U,
        'Mono_cam_cc_v': CC_V,
        'Mono_cam_distortions': [0.0, 0.0, 0.0, 0.0, 0.0],
        'Mono_cam_alpha_c': 0.0,
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


def write_params(path: str, calibration_source: str) -> None:
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(
            {'/**': {'ros__parameters': base_params(calibration_source)}}, f)


def make_camera_info(stamp, *, width=IMG_W, height=IMG_H, k1=0.0, k2=0.0,
                     p1=0.0, p2=0.0, k3=0.0, fx=FC_U, fy=FC_V,
                     cx=CC_U, cy=CC_V, frame=OPTICAL_FRAME,
                     model='plumb_bob') -> CameraInfo:
    info = CameraInfo()
    info.header.stamp = stamp
    info.header.frame_id = frame
    info.height = height
    info.width = width
    info.distortion_model = model
    info.d = [k1, k2, p1, p2, k3]
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


class Driver(Node):
    def __init__(self) -> None:
        super().__init__('a8_camera_info_driver')
        self.pub_imu = self.create_publisher(Imu, '/EKFslam/imu', 10)
        self.pub_alt = self.create_publisher(Altitude, '/EKFslam/altitude', 10)
        self.pub_spd = self.create_publisher(
            TwistWithCovarianceStamped, '/EKFslam/body_velocity', 10)
        self.pub_rng = self.create_publisher(Range, '/EKFslam/downward_range', 10)
        self.pub_img = self.create_publisher(Image, '/EKFslam/image', 10)
        self.pub_ci = self.create_publisher(CameraInfo, '/EKFslam/camera_info', 10)
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
            raise AssertionError('bad odom frames')

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

    def publish_nonvisual(self) -> None:
        now = self.get_clock().now().to_msg()
        q = _rpy_to_quat(0.02, -0.01, 0.05)
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

    def publish_image(self, *, frame_id: str = OPTICAL_FRAME,
                      width: int = IMG_W, height: int = IMG_H) -> None:
        now = self.get_clock().now().to_msg()
        img = Image()
        img.header.stamp = now
        img.header.frame_id = frame_id
        img.height = height
        img.width = width
        img.encoding = 'mono8'
        img.step = width
        # Fixture texture only defined for default size; pad/truncate otherwise.
        data = make_textured_frame(self.frame_idx)
        need = width * height
        if len(data) >= need:
            img.data = data[:need]
        else:
            img.data = data + bytes(need - len(data))
        self.frame_idx += 1
        self.pub_img.publish(img)

    def publish_burst(self, n: int = 40, *, with_ci: bool = False,
                      ci_factory=None) -> None:
        for _ in range(n):
            self.publish_nonvisual()
            if with_ci:
                now = self.get_clock().now().to_msg()
                self.pub_ci.publish(
                    ci_factory(now) if ci_factory else make_camera_info(now))
            self.publish_image()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

    def spin_for(self, sec: float) -> None:
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.1)

    def reset_counts(self) -> None:
        self.odom_n = self.path_n = self.lm_n = self.rs_n = 0
        self.last_path_len = self.last_lm_width = 0


def wait_for_node(timeout: float = 25.0) -> bool:
    n = Node('a8_waiter')
    try:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if 'EKFslam' in n.get_node_names():
                return True
            rclpy.spin_once(n, timeout_sec=0.1)
        return False
    finally:
        n.destroy_node()


def launch_localslam(params_path: str, log_path: str,
                     calibration_source: str) -> subprocess.Popen:
    cmd = [
        'ros2', 'launch', 'localslam', 'localslam.launch.py',
        f'params_file:={params_path}',
        'use_sim_time:=false',
        'sensor_input_mode:=standard',
        f'calibration_source:={calibration_source}',
        'visual_update:=true',
        'ground_station_range_update:=false',
    ]
    logf = open(log_path, 'w', buffering=1)
    return subprocess.Popen(
        cmd, stdout=logf, stderr=subprocess.STDOUT, start_new_session=True)


def kill_launch(proc: Optional[subprocess.Popen]) -> None:
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
    time.sleep(1.0)


def run_mode(log_dir: str, calibration_source: str,
             with_ci: bool, extra=None) -> dict:
    params_path = os.path.join(log_dir, f'a8_{calibration_source}_params.yaml')
    log_path = os.path.join(log_dir, f'a8_{calibration_source}_launch.log')
    write_params(params_path, calibration_source)
    proc = launch_localslam(params_path, log_path, calibration_source)
    print(f'[{calibration_source}] launch pid={proc.pid} log={log_path}', flush=True)
    try:
        if not wait_for_node(30.0):
            raise RuntimeError('EKFslam missing; see ' + log_path)
        driver = Driver()
        try:
            driver.wait_srvs()
            driver.call_enable(True)
            if extra:
                extra(driver)
            else:
                driver.publish_burst(50, with_ci=with_ci)
                driver.spin_for(3.0)
            return {
                'odom': driver.odom_n,
                'path': driver.path_n,
                'path_len': driver.last_path_len,
                'lm': driver.lm_n,
                'lm_width': driver.last_lm_width,
                'rs': driver.rs_n,
            }
        finally:
            driver.destroy_node()
    finally:
        kill_launch(proc)


def main() -> int:
    log_dir = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'log_a8'))
    os.makedirs(log_dir, exist_ok=True)
    os.chdir('/tmp')
    try:
        print('=== A8 CameraInfo calibration smoke ===', flush=True)
        rclpy.init()

        # 1) YAML baseline
        yaml_stats = run_mode(log_dir, 'yaml', with_ci=False)
        print(f'yaml stats: {yaml_stats}', flush=True)
        if yaml_stats['odom'] < 1 or yaml_stats['rs'] < 1:
            raise RuntimeError('YAML mode produced no odometry/robotstate')

        # 2) Equivalent CameraInfo
        ci_stats = run_mode(log_dir, 'camera_info', with_ci=True)
        print(f'camera_info stats: {ci_stats}', flush=True)
        if ci_stats['odom'] < 1 or ci_stats['rs'] < 1:
            raise RuntimeError('camera_info mode produced no odometry/robotstate')

        # 3) Missing CameraInfo: nonvisual still works; images bounded/dropped
        def missing_ci(driver: Driver) -> None:
            for _ in range(15):
                driver.publish_nonvisual()
                driver.publish_image()
                rclpy.spin_once(driver, timeout_sec=0.0)
                time.sleep(0.05)
            driver.spin_for(1.0)
            if driver.odom_n < 1:
                raise RuntimeError('missing-CI: expected nonvisual odometry')
            # Now supply valid CI and visual
            driver.publish_burst(40, with_ci=True)
            driver.spin_for(2.0)

        miss = run_mode(log_dir, 'camera_info', with_ci=False, extra=missing_ci)
        print(f'missing-then-valid CI stats: {miss}', flush=True)

        # 4) Invalid distortion / wrong size / changed-without-reset
        def edge_cases(driver: Driver) -> None:
            # Invalid tangential — should not unlock visual
            now = driver.get_clock().now().to_msg()
            driver.pub_ci.publish(make_camera_info(now, p1=0.05))
            for _ in range(8):
                driver.publish_nonvisual()
                driver.publish_image()
                time.sleep(0.05)
            # Valid first calibration
            driver.publish_burst(20, with_ci=True)
            driver.spin_for(1.0)
            odom_before = driver.odom_n
            # Changed fx without reset — must be ignored (still running)
            def bad_ci(stamp):
                return make_camera_info(stamp, fx=FC_U * 1.5)
            driver.publish_burst(10, with_ci=True, ci_factory=bad_ci)
            # Wrong image size — rejected
            for _ in range(5):
                driver.publish_nonvisual()
                now = driver.get_clock().now().to_msg()
                driver.pub_ci.publish(make_camera_info(now))
                driver.publish_image(width=100, height=80)
                time.sleep(0.05)
            # Reset then accept new geometry
            driver.call_reset()
            driver.publish_burst(25, with_ci=True, ci_factory=bad_ci)
            driver.spin_for(2.0)
            if driver.odom_n <= odom_before:
                raise RuntimeError('edge: no odometry after reset+new CI')

        edge = run_mode(log_dir, 'camera_info', with_ci=False, extra=edge_cases)
        print(f'edge-case stats: {edge}', flush=True)

        print('A8_CAMERA_INFO_SMOKE_OK', flush=True)
        return 0
    except Exception as exc:
        print(f'A8_CAMERA_INFO_SMOKE_FAIL: {exc}', file=sys.stderr, flush=True)
        traceback.print_exc()
        return 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        subprocess.run(
            ['pkill', '-f', 'localslam_container|localslam.launch'],
            check=False, capture_output=True)


if __name__ == '__main__':
    sys.exit(main())
