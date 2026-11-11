#!/usr/bin/env python3
"""G6b.1: order-independent feedback completion + abort-hold smoke.

Part A — real Local SLAM + fake Control (GT early FeedbackAccepted):
  1. prepare_pause (plan was active)
  2. LSposUpdate → CorrectionApplied
  3. Fake Control publishes CorrectionFeedbackAccepted immediately
  4. Assert FeedbackAccepted observed before/without barrier timeout
  5. release with resume → plan may resume; zero unintended cmd_vel
     while paused (only zeros)

Part B — abort without resume_requested keeps hold semantics:
  prepare → abort(resume=false) → plan_resumed must be false;
  assert FakeControl did not auto-resume.

Part C — rapid repeated applied transactions with early ack (ordering).

Shipped Apply_global_corrections remains false; this smoke enables
nothing in YAML — it drives LSposUpdate directly like G6a.
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
from typing import List, Optional

import yaml

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool
from unique_identifier_msgs.msg import UUID
from interfaces.msg import (
    EstimatorReference,
    CorrectionApplied,
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


def _uuid(fill: int = 1) -> UUID:
    u = UUID()
    u.uuid = [fill] * 16
    u.uuid[6] = 0x40
    u.uuid[8] = 0x80
    return u


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
        'GS_xy_update_max_delta': 5.0,
        'GS_z_update_max_delta': 0.1,
        'CL_xy_update': True,
        'CL_z_update': False,
        'CL_xy_update_max_delta': 15.0,
        'CL_z_update_max_delta': 1.0,
    }
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump({'/**': {'ros__parameters': params}}, f)


class FakeControl(Node):
    """GT feedback: ACK CorrectionApplied immediately (early vs LS caller)."""

    def __init__(self) -> None:
        super().__init__('g6b1_fake_control')
        self.cb = ReentrantCallbackGroup()
        self.paused = False
        self.plan_was_active = False
        self.plan_active = True
        self.txn = 0
        self.owner = 0
        self.session: Optional[UUID] = None
        self.epoch = 0
        self.releases = 0
        self.aborts = 0
        self.abort_resumes = 0
        self.release_resumes = 0
        self.feedback_pubs = 0
        self.feedback_ids: List[int] = []
        self.cmd_vel_nonzero_while_paused = 0
        self.cmd_vel_any = 0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=8,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_fb = self.create_publisher(
            CorrectionFeedbackAccepted, 'correction_feedback_accepted', qos)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(
            CorrectionApplied, '/EKFslam/correction_applied',
            self._on_applied, qos, callback_group=self.cb)
        self.create_subscription(
            Twist, 'cmd_vel', self._on_cmd, 10, callback_group=self.cb)
        self.create_service(
            CorrectionCoordinate, 'control_correction_coordinate',
            self._coord, callback_group=self.cb)
        self.create_service(
            SimpleServ, 'control_run_service', self._legacy, callback_group=self.cb)

    def _on_cmd(self, msg: Twist) -> None:
        mag = (
            abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.linear.z) +
            abs(msg.angular.z))
        if mag > 1e-9:
            self.cmd_vel_any += 1
            if self.paused:
                self.cmd_vel_nonzero_while_paused += 1

    def _on_applied(self, msg: CorrectionApplied) -> None:
        if not msg.applied:
            return
        out = CorrectionFeedbackAccepted()
        out.header = msg.header
        out.session_id = msg.session_id
        out.epoch_id = msg.epoch_id
        out.correction_id = msg.correction_id
        out.snapshot_seq = msg.snapshot_seq
        out.feedback_mode = 'gt'
        self.pub_fb.publish(out)
        self.feedback_pubs += 1
        self.feedback_ids.append(int(msg.correction_id))
        self.get_logger().info(
            f'early FeedbackAccepted correction_id={msg.correction_id}')

    def _legacy(self, req, resp):
        resp.response = True
        return resp

    def _coord(self, req, resp):
        resp.success = True
        resp.reason = ''
        resp.cmd_source_quiescent = True
        resp.plan_was_active = self.plan_was_active
        resp.plan_resumed = False
        resp.active_transaction_id = req.transaction_id
        resp.active_owner_generation = req.owner_generation
        if req.operation == 'prepare_pause':
            self.paused = True
            self.plan_was_active = self.plan_active
            self.plan_active = False
            self.txn = req.transaction_id
            self.owner = req.owner_generation
            self.session = req.session_id
            self.epoch = req.epoch_id
            self.pub_cmd.publish(Twist())  # zero while paused
            resp.state = 'paused'
            resp.plan_was_active = self.plan_was_active
        elif req.operation == 'release':
            self.releases += 1
            if req.resume_plan_if_owned and self.plan_was_active:
                resp.plan_resumed = True
                self.release_resumes += 1
                self.plan_active = True
            self.paused = False
            self.plan_was_active = False
            resp.state = 'idle'
        elif req.operation == 'abort':
            self.aborts += 1
            if req.resume_plan_if_owned and self.plan_was_active:
                resp.plan_resumed = True
                self.abort_resumes += 1
                self.plan_active = True
            self.paused = False
            self.plan_was_active = False
            resp.state = 'idle'
            resp.reason = 'aborted'
        else:
            resp.state = 'idle'
        return resp


class Probe(Node):
    def __init__(self) -> None:
        super().__init__('g6b1_probe')
        self.cb = ReentrantCallbackGroup()
        qos_ref = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos_fb = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=16,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.last_ref: Optional[EstimatorReference] = None
        self.accepted_ids: List[int] = []
        self.applied: List[CorrectionApplied] = []
        self.create_subscription(
            EstimatorReference, '/EKFslam/estimator_reference',
            self._on_ref, qos_ref, callback_group=self.cb)
        self.create_subscription(
            CorrectionApplied, '/EKFslam/correction_applied',
            self._on_applied, qos_fb, callback_group=self.cb)
        self.create_subscription(
            CorrectionFeedbackAccepted, 'correction_feedback_accepted',
            self._on_fb, qos_fb, callback_group=self.cb)
        self.pub_imu = self.create_publisher(Imu, '/EKFslam/imu', 10)
        self.cli_enable = self.create_client(SetBool, '/EKFslam/enable')
        self.cli_pos = self.create_client(LSposUpdate, 'local_slam_pos_update')
        self.cli_coord = self.create_client(
            CorrectionCoordinate, 'control_correction_coordinate')

    def _on_ref(self, msg: EstimatorReference) -> None:
        self.last_ref = msg

    def _on_applied(self, msg: CorrectionApplied) -> None:
        self.applied.append(msg)

    def _on_fb(self, msg: CorrectionFeedbackAccepted) -> None:
        self.accepted_ids.append(int(msg.correction_id))

    def wait_srvs(self, timeout: float = 30.0) -> None:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if (self.cli_enable.service_is_ready() and
                    self.cli_pos.service_is_ready() and
                    self.cli_coord.service_is_ready()):
                return
            time.sleep(0.05)
        raise RuntimeError('services missing')

    def call_enable(self, on: bool) -> None:
        req = SetBool.Request()
        req.data = on
        fut = self.cli_enable.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5:
            time.sleep(0.02)
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
            time.sleep(0.02)

    def wait_ref(self, timeout: float = 20.0) -> EstimatorReference:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.last_ref is not None:
                return self.last_ref
            time.sleep(0.05)
        raise RuntimeError('no EstimatorReference')

    def coord(self, op: str, tid: int, owner: int, session: UUID, epoch: int,
              resume: bool = False) -> CorrectionCoordinate.Response:
        req = CorrectionCoordinate.Request()
        req.operation = op
        req.transaction_id = tid
        req.owner_generation = owner
        req.session_id = session
        req.epoch_id = epoch
        req.correction_type = 'bundle_adjustment'
        req.resume_plan_if_owned = resume
        req.signal_close_loop = False
        fut = self.cli_coord.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5:
            time.sleep(0.02)
        if not fut.done():
            raise RuntimeError(f'coord {op} timeout')
        return fut.result()

    def ls_update(self, ref: EstimatorReference, cid: int,
                  dx: float = 0.05, dy: float = -0.02) -> LSposUpdate.Response:
        req = LSposUpdate.Request()
        req.delta_x = dx
        req.delta_y = dy
        req.delta_z = 0.0
        req.type = 'bundle_adjustment'
        req.session_id = ref.session_id
        req.epoch_id = ref.epoch_id
        req.keyframe_id = 1
        req.correction_id = cid
        fut = self.cli_pos.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5:
            time.sleep(0.02)
        if not fut.done():
            raise RuntimeError('LSposUpdate timeout')
        return fut.result()


def main() -> int:
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    log_dir = os.path.join(root, 'log', 'g6b1_smoke')
    os.makedirs(log_dir, exist_ok=True)

    domain = os.environ.get('G6B1_SMOKE_DOMAIN_ID', str(60 + (os.getpid() % 30)))
    os.environ['ROS_DOMAIN_ID'] = domain
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = domain
    print(f'G6b.1 smoke ROS_DOMAIN_ID={domain}')

    rclpy.init()
    fake = FakeControl()
    probe = Probe()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(fake)
    ex.add_node(probe)
    import threading
    thr = threading.Thread(target=ex.spin, daemon=True)
    thr.start()

    params_path = os.path.join(tempfile.mkdtemp(prefix='g6b1_'), 'ls.yaml')
    write_ls_params(params_path)
    ls_cmd = [
        'ros2', 'launch', 'localslam', 'localslam.launch.py',
        f'params_file:={params_path}',
    ]
    stdout_f = open(os.path.join(log_dir, 'ls.stdout'), 'w')
    ls_proc = subprocess.Popen(
        ls_cmd, cwd=root, env=env, stdout=stdout_f, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid)
    try:
        probe.wait_srvs(40.0)
        probe.call_enable(True)
        probe.publish_init(60)
        ref = probe.wait_ref(25.0)
        print(f'ref session ok epoch={ref.epoch_id}')

        owner = 42
        timeouts = 0
        releases_ok = 0

        # --- Part A/C: repeated coordinated applied + early feedback ---
        for i in range(1, 6):
            tid = 1000 + i
            n_fb_before = len(probe.accepted_ids)
            resp_p = probe.coord(
                'prepare_pause', tid, owner, ref.session_id, ref.epoch_id)
            if not resp_p.success:
                print(f'FAIL prepare_pause i={i}: {resp_p.reason}')
                return 1
            # Nonzero cmd while paused must stay 0
            t_apply0 = time.time()
            ls_resp = probe.ls_update(ref, tid)
            if not ls_resp.applied:
                print(f'FAIL LS not applied i={i} outcome={ls_resp.outcome}')
                return 1
            # Wait for FeedbackAccepted (must arrive without 5s GS-style barrier)
            deadline = time.time() + 2.0
            got = False
            while time.time() < deadline:
                if any(x == tid for x in probe.accepted_ids[n_fb_before:]):
                    got = True
                    break
                time.sleep(0.01)
            if not got:
                timeouts += 1
                print(f'FAIL early FeedbackAccepted missing tid={tid}')
                probe.coord(
                    'abort', tid, owner, ref.session_id, ref.epoch_id,
                    resume=False)
                continue
            # Normal release after feedback (mirrors GS ReadyToRelease)
            resp_r = probe.coord(
                'release', tid, owner, ref.session_id, ref.epoch_id,
                resume=True)
            if not resp_r.success:
                print(f'FAIL release tid={tid}: {resp_r.reason}')
                return 1
            releases_ok += 1
            dt = time.time() - t_apply0
            print(
                f'txn {tid}: applied+early_fb+release ok dt={dt:.3f}s '
                f'resumed={int(resp_r.plan_resumed)}')

        if timeouts != 0 or releases_ok != 5:
            print(
                f'FAIL Part A/C: releases_ok={releases_ok} timeouts={timeouts}')
            return 1

        # --- Part B: abort without resume must not resume ---
        tid = 2001
        fake.plan_active = True
        resp_p = probe.coord(
            'prepare_pause', tid, owner, ref.session_id, ref.epoch_id)
        if not resp_p.success:
            print(f'FAIL prepare for abort: {resp_p.reason}')
            return 1
        resp_a = probe.coord(
            'abort', tid, owner, ref.session_id, ref.epoch_id, resume=False)
        if resp_a is None or resp_a.plan_resumed:
            print(
                f'FAIL abort-without-resume resumed='
                f'{getattr(resp_a, "plan_resumed", None)}')
            return 1
        if fake.abort_resumes != 0:
            print(f'FAIL abort_resumes={fake.abort_resumes}')
            return 1
        print('abort-without-resume OK (plan_resumed=0)')

        if fake.cmd_vel_nonzero_while_paused != 0:
            print(
                f'FAIL nonzero cmd_vel while paused: '
                f'{fake.cmd_vel_nonzero_while_paused}')
            return 1

        print(
            f'G6b.1 smoke PASS releases={releases_ok} aborts={fake.aborts} '
            f'fb={fake.feedback_pubs} '
            f'cmd_nonzero_paused={fake.cmd_vel_nonzero_while_paused}')
        return 0
    finally:
        try:
            if ls_proc.poll() is None:
                try:
                    os.killpg(os.getpgid(ls_proc.pid), signal.SIGTERM)
                except Exception:
                    try:
                        ls_proc.terminate()
                    except Exception:
                        pass
                try:
                    ls_proc.wait(timeout=5)
                except Exception:
                    try:
                        os.killpg(os.getpgid(ls_proc.pid), signal.SIGKILL)
                    except Exception:
                        try:
                            ls_proc.kill()
                        except Exception:
                            pass
        except Exception:
            pass
        try:
            stdout_f.close()
        except Exception:
            pass
        try:
            ex.shutdown()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception:
        traceback.print_exc()
        sys.exit(1)
