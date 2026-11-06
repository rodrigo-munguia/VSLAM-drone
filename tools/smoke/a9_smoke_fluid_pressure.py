#!/usr/bin/env python3
"""A9: compare derived_altitude vs physically corresponding FluidPressure.

Uses independent ISA inverse (same constants as C++ helper docs) to synthesize
pressure from a relative altitude profile. Compares odometry z response under
identical non-altitude sensors. Not hardware validation.

Expects install_a9 (or overlay with localslam) sourced.
"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import time
import traceback
from typing import List, Optional, Tuple

import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from mavros_msgs.msg import Altitude
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure, Image, Imu, Range
from std_srvs.srv import SetBool, Trigger

# ISA troposphere constants (match pressure_altitude.hpp)
P0 = 101325.0
T0 = 288.15
L = 0.0065
G = 9.80665
R_SPEC = 287.05287

IMG_H, IMG_W = 240, 320
BODY_VX = 0.2
RANGE_M = 2.0
H0_ISA = 1000.0  # absolute ISA height of reference [m]
REL_PROFILE = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 2.5, 2.0, 1.5]  # +up [m]
# Absolute tracking lags the command under process noise; cross-mode agreement
# is the A9 contract check. Documented tolerances:
TOL_ABS_M = 0.85   # |odom_z - rel_cmd|
TOL_CROSS_M = 0.08  # |z_derived - z_pressure| at matching commands


def isa_pressure_from_height(h_m: float) -> float:
    """Independent inverse (Python) — not a copy of the C++ online path."""
    inner = 1.0 - (L * h_m) / T0
    if inner <= 0.0:
        raise ValueError('height out of troposphere')
    exp = G / (R_SPEC * L)
    return P0 * (inner ** exp)


def make_gray_frame() -> bytes:
    return bytes([128] * (IMG_H * IMG_W))


def _rpy_to_quat(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def write_params(path: str, altitude_mode: str) -> None:
    params = {
        'Sensor_input_mode': 'standard',
        'Altitude_input_mode': altitude_mode,
        'Altitude_update': True,
        'Visual_update': False,
        'Attitude_update': True,
        'Speed_update': True,
        'Ground_station_range_update': False,
        'Data_origin': 'standalone',
        'Calibration_source': 'yaml',
        'Prediction_model': 'noise_driven',
        'Max_freq_output': 50,
        'Robot_state_size': 13,
        'Mono_cam_img_rows': IMG_H,
        'Mono_cam_img_cols': IMG_W,
        'Sigma_att_update': 0.001,
        'Sigma_spd_update': 0.1,
        'Sigma_alt_update': 0.1,
        'Sigma_w': 0.1,
        'Sigma_a': 6.0,
        'Tau_r': 1.0,
        'Tau_p': 100.0,
        'Alt_h_2_d': -1,
        'Odometry_frame_id': 'odom',
        'Odometry_child_frame_id': 'base_link',
        'Pressure_reference_policy': 'first_valid',
        'Pressure_t0_K': T0,
        'Pressure_z_lever_arm_m': 0.0,
        'use_sim_time': False,
    }
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump({'/**': {'ros__parameters': params}}, f)


class Driver(Node):
    def __init__(self) -> None:
        super().__init__('a9_pressure_driver')
        self.pub_imu = self.create_publisher(Imu, '/EKFslam/imu', 10)
        self.pub_spd = self.create_publisher(
            TwistWithCovarianceStamped, '/EKFslam/body_velocity', 10)
        self.pub_rng = self.create_publisher(Range, '/EKFslam/downward_range', 10)
        self.pub_img = self.create_publisher(Image, '/EKFslam/image', 10)
        self.pub_alt = self.create_publisher(Altitude, '/EKFslam/altitude', 10)
        self.pub_fp = self.create_publisher(
            FluidPressure, '/EKFslam/pressure', 10)
        self.cli_enable = self.create_client(SetBool, '/EKFslam/enable')
        self.cli_reset = self.create_client(Trigger, '/EKFslam/reset')
        self.z_samples: List[Tuple[float, float]] = []  # (rel_cmd, odom_z)
        self.create_subscription(Odometry, '/EKFslam/odometry', self._on_odom, 10)
        self._last_z = 0.0

    def _on_odom(self, msg: Odometry) -> None:
        # Public odometry applies the A2 axis map; +up relative altitude
        # appears as increasing public z (not raw EKF NED z).
        self._last_z = msg.pose.pose.position.z

    def wait_srvs(self, timeout: float = 20.0) -> None:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.cli_enable.service_is_ready() and self.cli_reset.service_is_ready():
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError('services missing')

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

    def publish_common(self) -> None:
        now = self.get_clock().now().to_msg()
        q = _rpy_to_quat(0.0, 0.0, 0.0)
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'imu_link'
        imu.orientation.x, imu.orientation.y = q[0], q[1]
        imu.orientation.z, imu.orientation.w = q[2], q[3]
        imu.orientation_covariance[0] = 0.0
        imu.linear_acceleration.z = 9.81
        self.pub_imu.publish(imu)
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
        img.data = make_gray_frame()
        self.pub_img.publish(img)

    def run_profile(self, mode: str) -> List[Tuple[float, float]]:
        self.z_samples.clear()
        # Warm-up / latch reference at relative 0
        for _ in range(15):
            self.publish_common()
            now = self.get_clock().now().to_msg()
            if mode == 'derived_altitude':
                alt = Altitude()
                alt.header.stamp = now
                alt.relative = 0.0
                self.pub_alt.publish(alt)
            else:
                fp = FluidPressure()
                fp.header.stamp = now
                fp.header.frame_id = 'baro_link'
                fp.fluid_pressure = isa_pressure_from_height(H0_ISA)
                fp.variance = 999.0  # must be ignored
                self.pub_fp.publish(fp)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

        for rel in REL_PROFILE:
            for _ in range(20):
                self.publish_common()
                now = self.get_clock().now().to_msg()
                if mode == 'derived_altitude':
                    alt = Altitude()
                    alt.header.stamp = now
                    alt.relative = float(rel)
                    self.pub_alt.publish(alt)
                else:
                    fp = FluidPressure()
                    fp.header.stamp = now
                    fp.header.frame_id = 'baro_link'
                    fp.fluid_pressure = isa_pressure_from_height(H0_ISA + rel)
                    fp.variance = 123.0
                    self.pub_fp.publish(fp)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.05)
            t0 = time.time()
            while time.time() - t0 < 0.5:
                rclpy.spin_once(self, timeout_sec=0.05)
            self.z_samples.append((rel, self._last_z))
        return list(self.z_samples)


def wait_for_node(timeout: float = 25.0) -> bool:
    n = Node('a9_waiter')
    try:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if 'EKFslam' in n.get_node_names():
                return True
            rclpy.spin_once(n, timeout_sec=0.1)
        return False
    finally:
        n.destroy_node()


def launch_mode(params_path: str, log_path: str, mode: str) -> subprocess.Popen:
    cmd = [
        'ros2', 'launch', 'localslam', 'localslam.launch.py',
        f'params_file:={params_path}',
        'use_sim_time:=false',
        'sensor_input_mode:=standard',
        f'altitude_input_mode:={mode}',
        'visual_update:=false',
        'ground_station_range_update:=false',
        'sigma_alt_update:=0.1',
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


def run_one(log_dir: str, mode: str) -> List[Tuple[float, float]]:
    params_path = os.path.join(log_dir, f'a9_{mode}_params.yaml')
    log_path = os.path.join(log_dir, f'a9_{mode}_launch.log')
    write_params(params_path, mode)
    proc = launch_mode(params_path, log_path, mode)
    print(f'[{mode}] pid={proc.pid} log={log_path}', flush=True)
    try:
        if not wait_for_node(30.0):
            raise RuntimeError('EKFslam missing; see ' + log_path)
        driver = Driver()
        try:
            driver.wait_srvs()
            driver.call_enable(True)
            samples = driver.run_profile(mode)
            driver.call_reset()
            # After reset, re-latch and check relative zero again
            for _ in range(10):
                driver.publish_common()
                now = driver.get_clock().now().to_msg()
                if mode == 'fluid_pressure':
                    fp = FluidPressure()
                    fp.header.stamp = now
                    fp.header.frame_id = 'baro_link'
                    fp.fluid_pressure = isa_pressure_from_height(H0_ISA + 50.0)
                    driver.pub_fp.publish(fp)
                else:
                    alt = Altitude()
                    alt.header.stamp = now
                    alt.relative = 0.0
                    driver.pub_alt.publish(alt)
                time.sleep(0.05)
                rclpy.spin_once(driver, timeout_sec=0.0)
            return samples
        finally:
            driver.destroy_node()
    finally:
        kill_launch(proc)


def main() -> int:
    log_dir = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'log_a9'))
    os.makedirs(log_dir, exist_ok=True)
    os.chdir('/tmp')
    try:
        print('=== A9 FluidPressure altitude smoke ===', flush=True)
        # Sanity: independent inverse at sea level
        assert abs(isa_pressure_from_height(0.0) - P0) < 1e-6

        rclpy.init()
        derived = run_one(log_dir, 'derived_altitude')
        pressure = run_one(log_dir, 'fluid_pressure')
        print(f'derived samples: {derived}', flush=True)
        print(f'pressure samples: {pressure}', flush=True)

        if len(derived) != len(pressure):
            raise RuntimeError('sample count mismatch')
        max_dz = 0.0
        for (rel_d, z_d), (rel_p, z_p) in zip(derived, pressure):
            if abs(rel_d - rel_p) > 1e-9:
                raise RuntimeError('profile mismatch')
            err_d = abs(z_d - rel_d)
            err_p = abs(z_p - rel_p)
            dz = abs(z_d - z_p)
            max_dz = max(max_dz, dz)
            if rel_d > 0.0 and z_d <= 0.0:
                raise RuntimeError(f'derived sign wrong at rel={rel_d}: z={z_d}')
            if rel_d > 0.0 and z_p <= 0.0:
                raise RuntimeError(f'pressure sign wrong at rel={rel_d}: z={z_p}')
            if err_d > TOL_ABS_M:
                raise RuntimeError(
                    f'derived z={z_d} vs expected {rel_d} (tol {TOL_ABS_M})')
            if err_p > TOL_ABS_M:
                raise RuntimeError(
                    f'pressure z={z_p} vs expected {rel_p} (tol {TOL_ABS_M})')
            if dz > TOL_CROSS_M:
                raise RuntimeError(
                    f'mode disagreement at rel={rel_d}: z_d={z_d} z_p={z_p} '
                    f'(tol {TOL_CROSS_M})')
        print(
            f'max |z_derived - z_pressure| = {max_dz:.4f} m '
            f'(cross tol {TOL_CROSS_M}; abs tol {TOL_ABS_M})',
            flush=True)
        print('A9_FLUID_PRESSURE_SMOKE_OK', flush=True)
        return 0
    except Exception as exc:
        print(f'A9_FLUID_PRESSURE_SMOKE_FAIL: {exc}', file=sys.stderr, flush=True)
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
