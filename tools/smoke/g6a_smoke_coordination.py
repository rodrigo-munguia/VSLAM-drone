#!/usr/bin/env python3
"""G6a: isolated correction↔control coordination smoke.

Uses real Local SLAM plus an in-process fake Control implementing
CorrectionCoordinate (same ownership/idempotency rules as g6_coord_policy).
Captures cmd_vel and legacy 'c' on fake endpoints — no Webots/vehicle.

Shipped Apply_global_corrections remains false; this smoke does not enable
integrated corrections.
"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import tempfile
import time
import traceback
from typing import Optional

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool
from unique_identifier_msgs.msg import UUID
from interfaces.msg import EstimatorReference, CorrectionApplied
from interfaces.srv import LSposUpdate, CorrectionCoordinate, SimpleServ

IMG_H, IMG_W = 240, 400


def _rpy_to_quat(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * cy,
    )


def write_ls_params(path: str) -> None:
    params = {
        'Sensor_input_mode': 'standard',
        'Altitude_input_mode': 'derived_altitude',
        'Data_origin': 'standalone',
        'Visual_update': False,
        'Attitude_update': True,
        'Altitude_update': False,
        'Speed_update': False,
        'Ground_station_range_update': False,
        'Prediction_model': 'noise_driven',
        'Max_freq_output': 50,
        'Robot_state_size': 13,
        'Mono_cam_img_rows': IMG_H,
        'Mono_cam_img_cols': IMG_W,
        'Mono_cam_fc_u': 200.0,
        'Mono_cam_fc_v': 200.0,
        'Mono_cam_cc_u': 200.0,
        'Mono_cam_cc_v': 120.0,
        'Mono_cam_distortions': [0.0] * 5,
        'Sigma_att_update': 0.001,
        'Sigma_w': 0.1,
        'Sigma_a': 6.0,
        'Tau_r': 1.0,
        'Tau_p': 100.0,
        'Stats': False,
        'Odometry_frame_id': 'odom',
        'Odometry_child_frame_id': 'base_link',
        'use_sim_time': False,
        'GS_xy_update': True,
        'GS_z_update': False,
        'GS_xy_update_max_delta': 1.0,
        'GS_z_update_max_delta': 0.1,
        'CL_xy_update': True,
        'CL_z_update': False,
        'CL_xy_update_max_delta': 15.0,
        'CL_z_update_max_delta': 1.0,
    }
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump({'/**': {'ros__parameters': params}}, f)


def uuid_eq(a: UUID, b: UUID) -> bool:
    return tuple(a.uuid) == tuple(b.uuid)


class Probe(Node):
    """Fake control + LS client. Plan starts idle (plan_was_active=false)."""

    def __init__(self) -> None:
        super().__init__('g6a_coord_probe')
        qos_ref = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.last_ref: Optional[EstimatorReference] = None
        self.last_corr: Optional[CorrectionApplied] = None
        self.cmd_vel_n = 0
        self.zero_cmd_n = 0
        self.legacy_c = 0
        self.plan_active = False  # simulate idle plan
        self.paused = False
        self.active_tid = 0
        self.active_owner = 0
        self.active_session: Optional[UUID] = None
        self.active_epoch = 0
        self.plan_was_active = False
        self.operator_superseded = False
        self.cl_flag = False

        self.create_subscription(
            EstimatorReference, '/EKFslam/estimator_reference', self._on_ref, qos_ref)
        self.create_subscription(
            CorrectionApplied, '/EKFslam/correction_applied', self._on_corr, qos_ref)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self.pub_imu = self.create_publisher(Imu, '/EKFslam/imu', 10)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cli_enable = self.create_client(SetBool, '/EKFslam/enable')
        self.cli_pos = self.create_client(LSposUpdate, 'local_slam_pos_update')
        self.create_service(
            CorrectionCoordinate, 'control_correction_coordinate', self._on_coord)
        self.create_service(SimpleServ, 'control_run_service', self._on_legacy)

    def _on_ref(self, msg: EstimatorReference) -> None:
        self.last_ref = msg

    def _on_corr(self, msg: CorrectionApplied) -> None:
        self.last_corr = msg

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd_vel_n += 1
        if (msg.linear.x == 0 and msg.linear.y == 0 and msg.linear.z == 0 and
                msg.angular.z == 0):
            self.zero_cmd_n += 1

    def _on_legacy(self, request, response):
        if request.cmd == ord('c') or request.cmd == 'c':
            self.legacy_c += 1
            self.cl_flag = True
        response.response = True
        return response

    def _on_coord(self, request, response):
        response.success = False
        response.state = 'idle'
        response.reason = ''
        response.plan_was_active = self.plan_was_active
        response.plan_resumed = False
        response.cmd_source_quiescent = True
        response.active_transaction_id = self.active_tid
        response.active_owner_generation = self.active_owner
        op = request.operation
        if op == 'prepare_pause':
            if request.transaction_id == 0 or request.owner_generation == 0:
                response.reason = 'missing_transaction_or_owner'
                return response
            if self.paused:
                same = (
                    self.active_tid == request.transaction_id and
                    self.active_owner == request.owner_generation and
                    self.active_epoch == request.epoch_id and
                    self.active_session is not None and
                    uuid_eq(self.active_session, request.session_id))
                if same:
                    response.success = True
                    response.state = 'paused'
                    response.cmd_source_quiescent = True
                    # Publish zero (quiescent cmd source)
                    self.pub_cmd.publish(Twist())
                    return response
                response.reason = 'conflicting_transaction'
                response.state = 'paused'
                return response
            self.paused = True
            self.plan_was_active = self.plan_active
            self.plan_active = False
            self.active_tid = request.transaction_id
            self.active_owner = request.owner_generation
            self.active_session = request.session_id
            self.active_epoch = request.epoch_id
            self.operator_superseded = False
            self.pub_cmd.publish(Twist())
            response.success = True
            response.state = 'paused'
            response.plan_was_active = self.plan_was_active
            response.cmd_source_quiescent = True
            response.active_transaction_id = self.active_tid
            response.active_owner_generation = self.active_owner
            return response
        if op == 'release':
            if not self.paused:
                response.reason = 'no_active_pause'
                return response
            same = (
                self.active_tid == request.transaction_id and
                self.active_owner == request.owner_generation and
                self.active_epoch == request.epoch_id and
                self.active_session is not None and
                uuid_eq(self.active_session, request.session_id))
            if not same:
                response.reason = 'transaction_mismatch'
                response.state = 'paused'
                return response
            resume = (
                request.resume_plan_if_owned and self.plan_was_active and
                not self.operator_superseded)
            if request.signal_close_loop:
                self.cl_flag = True
            self.paused = False
            if resume:
                self.plan_active = True
            response.success = True
            response.state = 'idle'
            response.plan_resumed = resume
            response.plan_was_active = self.plan_was_active
            self.plan_was_active = False
            return response
        if op == 'abort':
            if not self.paused:
                response.success = True
                response.reason = 'already_idle'
                return response
            same = (
                self.active_tid == request.transaction_id and
                self.active_owner == request.owner_generation)
            if not same:
                response.reason = 'transaction_mismatch'
                response.state = 'paused'
                return response
            self.paused = False
            self.plan_was_active = False
            response.success = True
            response.state = 'idle'
            response.reason = 'aborted'
            return response
        if op == 'query':
            response.success = True
            response.state = 'paused' if self.paused else 'idle'
            return response
        response.reason = 'unknown_operation'
        return response

    def wait_srvs(self, timeout: float = 30.0) -> None:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.cli_enable.service_is_ready() and self.cli_pos.service_is_ready():
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError('LS services missing')

    def call_enable(self, on: bool) -> None:
        req = SetBool.Request()
        req.data = on
        fut = self.cli_enable.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or not fut.result().success:
            raise RuntimeError('enable failed')

    def publish_init(self, n: int = 80) -> None:
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
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)

    def wait_ref(self, timeout: float = 40.0) -> EstimatorReference:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_ref is not None and self.last_ref.valid:
                return self.last_ref
        raise RuntimeError('no valid ref')

    def coord(self, op, session, epoch, tid, owner, signal_cl=False, resume=False):
        # Call our own service via client for realistic async path.
        cli = self.create_client(CorrectionCoordinate, 'control_correction_coordinate')
        t0 = time.time()
        while time.time() - t0 < 5.0:
            if cli.service_is_ready():
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        req = CorrectionCoordinate.Request()
        req.operation = op
        req.session_id = session
        req.epoch_id = epoch
        req.transaction_id = tid
        req.owner_generation = owner
        req.correction_type = 'bundle_adjustment'
        req.signal_close_loop = signal_cl
        req.resume_plan_if_owned = resume
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            raise RuntimeError(f'coord {op} timeout')
        return fut.result()

    def pos_update(self, session, epoch, cid):
        req = LSposUpdate.Request()
        req.delta_x = 0.05
        req.delta_y = -0.07
        req.delta_z = 0.04
        req.type = 'bundle_adjustment'
        req.session_id = session
        req.epoch_id = epoch
        req.keyframe_id = 1
        req.correction_id = cid
        fut = self.cli_pos.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            raise RuntimeError('pos update timeout')
        return fut.result()


def kill_proc(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=5)
    except Exception:
        try:
            proc.terminate()
            proc.wait(timeout=3)
        except Exception:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass


def run() -> int:
    domain = os.environ.get('G6A_SMOKE_DOMAIN_ID', str(50 + (os.getpid() % 40)))
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = domain
    os.environ['ROS_DOMAIN_ID'] = domain
    tmp = tempfile.mkdtemp(prefix='g6a_')
    ls_yaml = os.path.join(tmp, 'ls.yaml')
    write_ls_params(ls_yaml)

    ls_proc = None
    rclpy.init()
    probe = Probe()
    try:
        print(f'G6a smoke ROS_DOMAIN_ID={domain}')
        ls_proc = subprocess.Popen(
            ['ros2', 'launch', 'localslam', 'localslam.launch.py',
             f'params_file:={ls_yaml}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env,
            preexec_fn=os.setsid, cwd='/tmp')
        probe.wait_srvs()
        probe.call_enable(True)
        probe.publish_init(80)
        ref = probe.wait_ref()
        session = ref.session_id
        epoch = int(ref.epoch_id)
        owner = 0x6A01
        tid = (owner << 16) | 1

        # Simulate active plan then pause
        probe.plan_active = True
        r = probe.coord('prepare_pause', session, epoch, tid, owner)
        print(f'pause: success={r.success} quiescent={r.cmd_source_quiescent} '
              f'was_active={r.plan_was_active} reason={r.reason}')
        if not r.success or not r.cmd_source_quiescent:
            raise AssertionError('pause not acknowledged')
        if not r.plan_was_active:
            raise AssertionError('expected plan_was_active')
        if probe.zero_cmd_n < 1:
            raise AssertionError('expected zero cmd_vel on pause')

        r2 = probe.coord('prepare_pause', session, epoch, tid + 1, owner)
        if r2.success:
            raise AssertionError('conflict should reject')
        print(f'conflict rejected: {r2.reason}')

        resp = probe.pos_update(session, epoch, tid)
        print(f'apply: applied={resp.applied} outcome={resp.outcome}')
        if not resp.applied:
            raise AssertionError('expected applied')

        t0 = time.time()
        while time.time() - t0 < 5.0:
            rclpy.spin_once(probe, timeout_sec=0.05)
            if (probe.last_corr is not None and
                    probe.last_corr.correction_id == tid and
                    probe.last_corr.applied):
                break
        if probe.last_corr is None or probe.last_corr.correction_id != tid:
            raise AssertionError('CorrectionApplied barrier missing')
        if getattr(probe.last_corr, 'snapshot_seq', 0) == 0:
            raise AssertionError('CorrectionApplied missing snapshot_seq (G6b)')
        print(f'barrier OK snapshot_seq={probe.last_corr.snapshot_seq}')

        r3 = probe.coord(
            'release', session, epoch, tid, owner, signal_cl=False, resume=True)
        print(f'release: success={r3.success} resumed={r3.plan_resumed}')
        if not r3.success or not r3.plan_resumed:
            raise AssertionError('expected plan resume')
        if not probe.plan_active:
            raise AssertionError('plan_active not restored')

        # Idle plan: pause while plan_active=false → release must not resume
        owner2 = 0x6A02
        tid2 = (owner2 << 16) | 7
        probe.plan_active = False
        r4 = probe.coord('prepare_pause', session, epoch, tid2, owner2)
        if not r4.success:
            raise AssertionError(f'idle pause failed: {r4.reason}')
        if r4.plan_was_active:
            raise AssertionError('idle pause should report plan_was_active=false')
        r5 = probe.coord(
            'release', session, epoch, tid2, owner2, resume=True)
        if not r5.success:
            raise AssertionError(f'idle release failed: {r5.reason}')
        if r5.plan_resumed:
            raise AssertionError('idle plan must not resume')
        print('idle-plan remains idle OK')

        if probe.legacy_c != 0:
            raise AssertionError(f'unexpected legacy c={probe.legacy_c}')

        print('G6a coordination smoke PASS')
        return 0
    except Exception:
        traceback.print_exc()
        return 1
    finally:
        kill_proc(ls_proc)
        probe.destroy_node()
        rclpy.shutdown()
        time.sleep(0.5)


if __name__ == '__main__':
    sys.exit(run())
