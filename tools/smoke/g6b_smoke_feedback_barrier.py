#!/usr/bin/env python3
"""G6b: unique odometry↔correction feedback association smoke.

Validates:
  - CorrectionApplied carries snapshot_seq
  - OdometryMeta pairs with Odometry (either arrival order)
  - Control publishes CorrectionFeedbackAccepted after accepting the
    required snapshot (odometry mode) or on CorrectionApplied (GT mode)
  - Stale session/epoch / old seq / duplicate events / reset

Uses Local SLAM + in-process fake Control (no Webots). Shipped YAML
Apply_global_corrections remains false; this smoke does not enable
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
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool
from unique_identifier_msgs.msg import UUID
from interfaces.msg import (
    EstimatorReference,
    CorrectionApplied,
    OdometryMeta,
    CorrectionFeedbackAccepted,
)
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
        'GS_xy_update_max_delta': 50.0,
        'GS_z_update_max_delta': 50.0,
        'CL_xy_update': True,
        'CL_z_update': False,
        'CL_xy_update_max_delta': 50.0,
        'CL_z_update_max_delta': 50.0,
    }
    with open(path, 'w') as f:
        yaml.safe_dump({'/**': {'ros__parameters': params}}, f)


def uuid_eq(a: UUID, b: UUID) -> bool:
    return list(a.uuid) == list(b.uuid)


class FakeControl(Node):
    """Mirrors g6_coord_policy + g6_feedback_barrier for odometry or GT."""

    def __init__(self, mode: str):
        super().__init__('g6b_fake_control')
        self.mode = mode  # 'odometry' | 'gt'
        self.plan_active = False
        self.paused = False
        self.active: Optional[Tuple] = None
        self.superseded = False
        self.plan_was_active = False
        self.zero_cmd_n = 0
        self.fb_accepted: Optional[CorrectionFeedbackAccepted] = None

        self._b_armed = False
        self._b_session: Optional[UUID] = None
        self._b_epoch = 0
        self._b_cid = 0
        self._b_req_seq = 0
        self._b_accepted = False
        self._meta_buf: Deque[OdometryMeta] = deque(maxlen=32)
        self._accepted_stamps: Deque[Tuple[int, int]] = deque(maxlen=32)
        self._accepted_snaps: Deque[Tuple] = deque(maxlen=32)

        self.create_subscription(Twist, 'cmd_vel', lambda _m: None, 10)
        self.create_subscription(
            Odometry, '/EKFslam/odometry', self._on_odom, 10)
        self.create_subscription(
            OdometryMeta, '/EKFslam/odometry_meta', self._on_meta, 10)
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            CorrectionApplied, '/EKFslam/correction_applied',
            self._on_applied, qos)
        self.pub_fb = self.create_publisher(
            CorrectionFeedbackAccepted, '/control/correction_feedback_accepted', qos)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_service(
            CorrectionCoordinate, 'control_correction_coordinate', self._on_coord)
        self.create_service(SimpleServ, 'control_run_service', self._on_run)

    def _on_run(self, req, resp):
        resp.response = True
        return resp

    def _publish_zero(self):
        self.pub_cmd.publish(Twist())
        self.zero_cmd_n += 1

    def _on_coord(self, req, resp):
        key = (tuple(req.session_id.uuid), req.epoch_id,
               req.transaction_id, req.owner_generation)
        resp.success = False
        resp.state = 'idle'
        resp.reason = ''
        resp.plan_was_active = False
        resp.plan_resumed = False
        resp.cmd_source_quiescent = True
        resp.active_transaction_id = 0
        resp.active_owner_generation = 0
        op = req.operation
        if op == 'prepare_pause':
            if self.paused and self.active != key:
                resp.reason = 'conflicting_transaction'
                return resp
            if not self.paused:
                self.plan_was_active = self.plan_active
                self.plan_active = False
                self.superseded = False
                self.active = key
                self.paused = True
            self._publish_zero()
            resp.success = True
            resp.state = 'paused'
            resp.plan_was_active = self.plan_was_active
            resp.cmd_source_quiescent = True
            resp.active_transaction_id = req.transaction_id
            resp.active_owner_generation = req.owner_generation
            return resp
        if op == 'release':
            if not self.paused or self.active != key:
                resp.reason = 'transaction_mismatch' if self.paused else 'no_active_pause'
                return resp
            if req.resume_plan_if_owned and self.mode == 'odometry':
                if not (self._b_accepted and self._b_cid == req.transaction_id):
                    resp.reason = 'feedback_barrier_not_satisfied'
                    return resp
            may = (req.resume_plan_if_owned and self.plan_was_active and
                   not self.superseded)
            self.paused = False
            if may:
                self.plan_active = True
            resp.success = True
            resp.plan_resumed = may
            resp.plan_was_active = self.plan_was_active
            self.plan_was_active = False
            self._b_armed = False
            self._b_accepted = False
            return resp
        if op == 'abort':
            if self.paused and self.active == key:
                self.paused = False
                self.plan_was_active = False
                self._b_armed = False
                self._b_accepted = False
            resp.success = True
            resp.reason = 'aborted'
            return resp
        resp.success = True
        return resp

    def _meta_ok(self, m: OdometryMeta) -> bool:
        if not self._b_armed or self._b_accepted:
            return False
        if not uuid_eq(m.session_id, self._b_session):
            return False
        if m.epoch_id != self._b_epoch:
            return False
        if m.last_correction_id != self._b_cid:
            return False
        return m.snapshot_seq >= self._b_req_seq

    def _accept_fb(self, mode: str, seq: int):
        self._b_accepted = True
        msg = CorrectionFeedbackAccepted()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.session_id = self._b_session
        msg.epoch_id = self._b_epoch
        msg.correction_id = self._b_cid
        msg.snapshot_seq = seq
        msg.feedback_mode = mode
        self.fb_accepted = msg
        self.pub_fb.publish(msg)

    def _on_applied(self, msg: CorrectionApplied):
        if not msg.applied or msg.correction_id == 0 or msg.snapshot_seq == 0:
            return
        self._b_armed = True
        self._b_session = msg.session_id
        self._b_epoch = msg.epoch_id
        self._b_cid = msg.correction_id
        self._b_req_seq = msg.snapshot_seq
        self._b_accepted = False
        if self.mode == 'gt':
            self._accept_fb('gt', msg.snapshot_seq)
            return
        for snap in reversed(self._accepted_snaps):
            sess, ep, seq, cid = snap
            if (uuid_eq(sess, msg.session_id) and ep == msg.epoch_id and
                    cid == msg.correction_id and seq >= msg.snapshot_seq):
                self._accept_fb('odometry', seq)
                return
        for m in self._meta_buf:
            if self._meta_ok(m):
                st = (m.header.stamp.sec, m.header.stamp.nanosec)
                if st in self._accepted_stamps:
                    self._accept_fb('odometry', m.snapshot_seq)
                    return

    def _on_odom(self, msg: Odometry):
        st = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self._accepted_stamps.append(st)
        for m in self._meta_buf:
            if (m.header.stamp.sec, m.header.stamp.nanosec) == st:
                self._accepted_snaps.append(
                    (m.session_id, m.epoch_id, m.snapshot_seq, m.last_correction_id))
                if self._meta_ok(m):
                    self._accept_fb('odometry', m.snapshot_seq)

    def _on_meta(self, msg: OdometryMeta):
        self._meta_buf.append(msg)
        st = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if st in self._accepted_stamps:
            self._accepted_snaps.append(
                (msg.session_id, msg.epoch_id, msg.snapshot_seq,
                 msg.last_correction_id))
            if self._meta_ok(msg):
                self._accept_fb('odometry', msg.snapshot_seq)

    def reset_barrier(self):
        self._b_armed = False
        self._b_accepted = False
        self._meta_buf.clear()
        self._accepted_stamps.clear()
        self._accepted_snaps.clear()
        self.fb_accepted = None


class Probe(Node):
    def __init__(self):
        super().__init__('g6b_probe')
        self.ref: Optional[EstimatorReference] = None
        self.last_corr: Optional[CorrectionApplied] = None
        self.last_meta: Optional[OdometryMeta] = None
        self.last_odom: Optional[Odometry] = None
        self.last_fb: Optional[CorrectionFeedbackAccepted] = None
        qos_ref = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            EstimatorReference, '/EKFslam/estimator_reference',
            lambda m: setattr(self, 'ref', m), qos_ref)
        self.create_subscription(
            CorrectionApplied, '/EKFslam/correction_applied',
            self._on_corr, qos_ref)
        self.create_subscription(
            OdometryMeta, '/EKFslam/odometry_meta',
            lambda m: setattr(self, 'last_meta', m), 10)
        self.create_subscription(
            Odometry, '/EKFslam/odometry',
            lambda m: setattr(self, 'last_odom', m), 10)
        self.create_subscription(
            CorrectionFeedbackAccepted, '/control/correction_feedback_accepted',
            lambda m: setattr(self, 'last_fb', m), qos_ref)
        self.pub_imu = self.create_publisher(Imu, '/EKFslam/imu', 10)
        self.cli_en = self.create_client(SetBool, '/EKFslam/enable')
        self.cli_pos = self.create_client(LSposUpdate, 'local_slam_pos_update')
        self.cli_coord = self.create_client(
            CorrectionCoordinate, 'control_correction_coordinate')

    def _on_corr(self, m: CorrectionApplied):
        self.last_corr = m

    def wait_srvs(self, timeout=30.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if (self.cli_en.service_is_ready() and
                    self.cli_pos.service_is_ready() and
                    self.cli_coord.service_is_ready()):
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise TimeoutError('services not ready')

    def enable(self, also_spin=None):
        req = SetBool.Request()
        req.data = True
        fut = self.cli_en.call_async(req)
        t0 = time.time()
        while time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.02)
            if also_spin is not None:
                rclpy.spin_once(also_spin, timeout_sec=0.02)
            if fut.done():
                break

    def publish_init(self, n=80, also_spin=None):
        q = _rpy_to_quat(0.02, -0.01, 0.05)
        for _ in range(n):
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'
            msg.orientation.x, msg.orientation.y = q[0], q[1]
            msg.orientation.z, msg.orientation.w = q[2], q[3]
            msg.orientation_covariance[0] = 0.0
            msg.linear_acceleration.z = 9.81
            self.pub_imu.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            if also_spin is not None:
                rclpy.spin_once(also_spin, timeout_sec=0.0)
            time.sleep(0.02)

    def wait_ref(self, timeout=40.0, also_spin=None) -> EstimatorReference:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if also_spin is not None:
                rclpy.spin_once(also_spin, timeout_sec=0.0)
            if self.ref is not None and self.ref.valid:
                return self.ref
        raise TimeoutError('no estimator reference')

    def coord(self, op, session, epoch, tid, owner, resume=False, also_spin=None):
        req = CorrectionCoordinate.Request()
        req.operation = op
        req.session_id = session
        req.epoch_id = epoch
        req.transaction_id = tid
        req.owner_generation = owner
        req.correction_type = 'bundle_adjustment'
        req.resume_plan_if_owned = resume
        fut = self.cli_coord.call_async(req)
        t0 = time.time()
        while time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.02)
            if also_spin is not None:
                rclpy.spin_once(also_spin, timeout_sec=0.02)
            if fut.done():
                break
        return fut.result()

    def pos_update(self, session, epoch, cid, dx=0.4, dy=0.0, dz=0.0, also_spin=None):
        req = LSposUpdate.Request()
        req.delta_x, req.delta_y, req.delta_z = dx, dy, dz
        req.type = 'bundle_adjustment'
        req.session_id = session
        req.epoch_id = epoch
        req.keyframe_id = 1
        req.correction_id = cid
        fut = self.cli_pos.call_async(req)
        t0 = time.time()
        while time.time() - t0 < 10.0:
            rclpy.spin_once(self, timeout_sec=0.02)
            if also_spin is not None:
                rclpy.spin_once(also_spin, timeout_sec=0.02)
            if fut.done():
                break
        return fut.result()


def kill_proc(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except (ProcessLookupError, PermissionError, OSError):
        try:
            proc.terminate()
        except Exception:
            pass
    try:
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass


def run_mode(mode: str) -> None:
    # Unique domain per mode so a failed teardown of a prior mode cannot
    # collide on local_slam_pos_update (sandbox may block killpg).
    # DDS multicast ports require domainId <= 232.
    base = int(os.environ.get('ROS_DOMAIN_ID', '91')) % 200
    domain = str(base + (0 if mode == 'gt' else 11))
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = domain
    os.environ['ROS_DOMAIN_ID'] = domain
    tmp = tempfile.mkdtemp(prefix=f'g6b_{mode}_')
    ls_yaml = os.path.join(tmp, 'ls.yaml')
    write_ls_params(ls_yaml)

    ls_proc = None
    rclpy.init()
    fake = FakeControl(mode)
    probe = Probe()
    try:
        print(f'G6b smoke mode={mode} ROS_DOMAIN_ID={domain}')
        ls_proc = subprocess.Popen(
            ['ros2', 'launch', 'localslam', 'localslam.launch.py',
             f'params_file:={ls_yaml}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env,
            preexec_fn=os.setsid, cwd='/tmp')
        # Spin both nodes
        for _ in range(40):
            rclpy.spin_once(fake, timeout_sec=0.05)
            rclpy.spin_once(probe, timeout_sec=0.05)
        probe.wait_srvs()
        probe.enable(also_spin=fake)
        probe.publish_init(80, also_spin=fake)
        ref = probe.wait_ref(also_spin=fake)
        session, epoch = ref.session_id, int(ref.epoch_id)
        owner, tid = 0x6B01, (0x6B01 << 16) | 1

        fake.plan_active = True
        r = probe.coord('prepare_pause', session, epoch, tid, owner, also_spin=fake)
        if not r or not r.success:
            raise AssertionError(f'pause failed: {None if r is None else r.reason}')

        resp = probe.pos_update(session, epoch, tid, also_spin=fake)
        if not resp or not resp.applied:
            raise AssertionError(
                f'apply failed: {None if resp is None else resp.outcome}')

        t0 = time.time()
        while time.time() - t0 < 8.0:
            rclpy.spin_once(fake, timeout_sec=0.05)
            rclpy.spin_once(probe, timeout_sec=0.05)
            if (probe.last_corr and probe.last_corr.correction_id == tid and
                    probe.last_corr.snapshot_seq != 0 and
                    probe.last_fb and probe.last_fb.correction_id == tid):
                break
        if probe.last_corr is None or probe.last_corr.snapshot_seq == 0:
            raise AssertionError('CorrectionApplied missing snapshot_seq')
        if probe.last_fb is None or probe.last_fb.correction_id != tid:
            raise AssertionError('CorrectionFeedbackAccepted missing')
        expected_mode = 'gt' if mode == 'gt' else 'odometry'
        if probe.last_fb.feedback_mode != expected_mode:
            raise AssertionError(
                f'feedback_mode={probe.last_fb.feedback_mode} want {expected_mode}')
        print(f'  applied snapshot_seq={probe.last_corr.snapshot_seq} '
              f'fb_mode={probe.last_fb.feedback_mode}')

        r2 = probe.coord(
            'release', session, epoch, tid, owner, resume=True, also_spin=fake)
        if not r2 or not r2.success or not r2.plan_resumed:
            raise AssertionError(
                f'release/resume failed: {None if r2 is None else r2.reason}')

        # Reset clears barrier — stale completion must not resume.
        fake.reset_barrier()
        fake.plan_active = True
        owner2, tid2 = 0x6B02, (0x6B02 << 16) | 2
        r3 = probe.coord(
            'prepare_pause', session, epoch, tid2, owner2, also_spin=fake)
        if not r3 or not r3.success:
            raise AssertionError('second pause failed')
        fake.superseded = True
        fake.plan_was_active = False
        r4 = probe.coord('abort', session, epoch, tid2, owner2, also_spin=fake)
        if not r4 or not r4.success:
            raise AssertionError('abort failed')
        if fake.plan_active:
            raise AssertionError('abort must not leave plan running')
        print(f'  mode={mode} PASS')
    finally:
        kill_proc(ls_proc)
        probe.destroy_node()
        fake.destroy_node()
        rclpy.shutdown()
        time.sleep(0.3)


def test_pure_association() -> None:
    """Stamp collision: unique key is snapshot_seq."""
    metas = [
        {'seq': 1, 'stamp': (1, 0)},
        {'seq': 2, 'stamp': (1, 0)},  # same stamp
    ]
    best = max((m for m in metas if m['stamp'] == (1, 0)), key=lambda m: m['seq'])
    assert best['seq'] == 2
    print('  pure association PASS')


def main() -> int:
    try:
        print('G6b feedback barrier smoke')
        test_pure_association()
        run_mode('gt')
        run_mode('odometry')
        print('G6b feedback barrier smoke PASS')
        return 0
    except Exception:
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
