#!/usr/bin/env python3
"""G3: Local+Global identity smoke (corrections remain off).

Exercises EstimatorReference session/epoch propagation, keyframe admission,
Local reset/re-init, Local component recreation (new session), Global restart,
and cleanup with both GS workers. Apply_global_corrections stays false.

Does not start Webots / launch2 / enable vehicle-affecting corrections.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import tempfile
import time
import traceback
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Transform
from std_srvs.srv import SetBool, Trigger
from unique_identifier_msgs.msg import UUID
from interfaces.msg import Kf, EstimatorReference
from interfaces.srv import LSposUpdate, SimpleServ

IMG_H, IMG_W = 240, 400  # match globalslam_params / typical mars YAML cols
TIMEOUT_S = 120.0


def uuid_to_tuple(u: UUID) -> Tuple[int, ...]:
    return tuple(int(x) for x in u.uuid)


def make_gray_frame() -> bytes:
    buf = bytearray(IMG_H * IMG_W)
    for v in range(IMG_H):
        for u in range(IMG_W):
            buf[v * IMG_W + u] = ((u // 16) + (v // 16)) % 2 * 200 + 20
    return bytes(buf)


def make_kf(
    node: Node,
    session: UUID,
    epoch: int,
    kid: int,
    stream: int,
    stamp_valid: bool = True,
) -> Kf:
    msg = Kf()
    msg.session_id = session
    msg.epoch_id = epoch
    msg.keyframe_id = kid
    msg.source_stamp_valid = stamp_valid
    msg.stream = stream
    if stamp_valid:
        msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'internal_nav_cam'
    msg.img.header = msg.header
    msg.img.height = IMG_H
    msg.img.width = IMG_W
    msg.img.encoding = 'mono8'
    msg.img.step = IMG_W
    msg.img.data = make_gray_frame()
    identity = Transform()
    identity.rotation.w = 1.0
    msg.r2c = identity
    msg.n2r = identity
    return msg


class Probe(Node):
    def __init__(self) -> None:
        super().__init__('g3_identity_probe')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.last_ref: Optional[EstimatorReference] = None
        self.ref_n = 0
        self.create_subscription(
            EstimatorReference, '/EKFslam/estimator_reference', self._on_ref, qos)
        self.pub_kf = self.create_publisher(Kf, 'Kf_topic', 10)
        self.pub_kf_cl = self.create_publisher(Kf, 'Kf_cl_topic', 10)
        self.cli_ls_enable = self.create_client(SetBool, '/EKFslam/enable')
        self.cli_ls_reset = self.create_client(Trigger, '/EKFslam/reset')
        self.cli_gs_enable = self.create_client(SetBool, '/gslam/enable')
        self.cli_gs_reset = self.create_client(Trigger, '/gslam/reset')
        self.ls_n = 0
        self.ctrl_c_n = 0
        self.create_service(LSposUpdate, 'local_slam_pos_update', self._on_ls)
        self.create_service(SimpleServ, 'control_run_service', self._on_ctrl)

    def _on_ref(self, msg: EstimatorReference) -> None:
        self.last_ref = msg
        self.ref_n += 1

    def _on_ls(self, request, response):
        self.ls_n += 1
        response.accepted = False
        response.applied = False
        response.outcome = 'rejected'
        response.reason = 'smoke_sink'
        return response

    def _on_ctrl(self, request, response):
        if request.cmd == ord('c'):
            self.ctrl_c_n += 1
        response.response = True
        return response

    def wait_ref(self, timeout: float, pred, spin_once=None) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if spin_once is not None:
                spin_once()
            else:
                rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_ref is not None and pred(self.last_ref):
                return True
        return False


def launch_pkg(
    pkg: str,
    launch: str,
    env: dict,
    params: Optional[str] = None,
    extra_args: Optional[List[str]] = None,
) -> subprocess.Popen:
    cmd = ['ros2', 'launch', pkg, launch]
    if params:
        cmd.append(f'params_file:={params}')
    if extra_args:
        cmd.extend(extra_args)
    # DEVNULL avoids PIPE-fill deadlock when launch is chatty.
    return subprocess.Popen(
        cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env,
        preexec_fn=os.setsid, cwd='/tmp')


# Isolate from any live experiment sensors on /EKFslam/*.
_LS_ISOLATED_SENSORS = [
    'image_topic:=/g3_smoke/unused/image',
    'camera_info_topic:=/g3_smoke/unused/camera_info',
    'imu_topic:=/g3_smoke/unused/imu',
    'body_velocity_topic:=/g3_smoke/unused/body_velocity',
    'downward_range_topic:=/g3_smoke/unused/downward_range',
    'altitude_topic:=/g3_smoke/unused/altitude',
    'pressure_topic:=/g3_smoke/unused/pressure',
]


def kill_proc(proc: Optional[subprocess.Popen], force: bool = False) -> None:
    if proc is None or proc.poll() is not None:
        return
    sig = signal.SIGKILL if force else signal.SIGINT
    try:
        os.killpg(os.getpgid(proc.pid), sig)
    except (ProcessLookupError, PermissionError, OSError):
        try:
            proc.terminate()
        except Exception:
            return
    try:
        proc.wait(timeout=5 if force else 8)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass


def call_bool(node: Node, cli, value: bool, timeout: float = 5.0) -> bool:
    req = SetBool.Request()
    req.data = value
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.done() and bool(fut.result() and fut.result().success)


def call_trig(node: Node, cli, timeout: float = 5.0) -> bool:
    fut = cli.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.done() and bool(fut.result() and fut.result().success)


def write_ls_params(path: str) -> None:
    # Minimal standalone LS params (yaml Calibration_source).
    text = f"""
/lslam:
  ros__parameters:
    Sensor_input_mode: standard
    Altitude_input_mode: derived_altitude
    Calibration_source: yaml
    Data_origin: standalone
    Visual_update: false
    # Keep Attitude_update true so system_init waits for attitude (no auto-init).
    Attitude_update: true
    Altitude_update: false
    Speed_update: false
    Ground_station_range_update: false
    Prediction_model: noise_driven
    Max_freq_output: 50
    Robot_state_size: 13
    Mono_cam_frame_period: 0.05
    Mono_cam_img_rows: {IMG_H}
    Mono_cam_img_cols: {IMG_W}
    Mono_cam_fc_u: 200.0
    Mono_cam_fc_v: 200.0
    Mono_cam_cc_u: 200.0
    Mono_cam_cc_v: 120.0
    Mono_cam_distortions: [0.0, 0.0, 0.0, 0.0, 0.0]
    Mono_cam_alpha_c: 0.0
    Odometry_frame_id: odom
    Odometry_child_frame_id: base_link
    use_sim_time: false
"""
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)


def main() -> int:
    t0 = time.time()
    install = os.environ.get(
        'EKF_INSTALL_BASE',
        os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            'install'))
    env = os.environ.copy()
    # Isolate from any live experiment DDS participants on the machine.
    env['ROS_DOMAIN_ID'] = env.get('G3_SMOKE_DOMAIN_ID', '91')
    procs: List[subprocess.Popen] = []
    # rclpy must see the same domain as launched nodes.
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    rclpy.init()
    probe = Probe()
    execu = SingleThreadedExecutor()
    execu.add_node(probe)

    def spin_for(seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            execu.spin_once(timeout_sec=0.05)

    try:
        with tempfile.TemporaryDirectory(prefix='g3_smoke_') as td:
            ls_params = os.path.join(td, 'ls.yaml')
            write_ls_params(ls_params)

            # --- Local SLAM instance A ---
            p_ls = launch_pkg(
                'localslam', 'localslam.launch.py', env, ls_params,
                _LS_ISOLATED_SENSORS)
            procs.append(p_ls)
            spin_for(2.0)
            def _spin():
                execu.spin_once(timeout_sec=0.05)

            if not probe.wait_ref(20.0, lambda m: m is not None, _spin):
                raise RuntimeError('no EstimatorReference from Local SLAM')
            # Initial invalid before enable/init is expected; session must be non-nil.
            session_a = uuid_to_tuple(probe.last_ref.session_id)
            if session_a == tuple([0] * 16):
                raise RuntimeError('nil session_id on EstimatorReference')
            epoch_a0 = probe.last_ref.epoch_id

            # --- Global SLAM ---
            p_gs = launch_pkg('globalslam', 'globalslam.launch.py', env)
            procs.append(p_gs)
            spin_for(2.0)
            t_wait = time.time()
            while time.time() - t_wait < 20.0:
                if probe.cli_gs_enable.wait_for_service(timeout_sec=0.5):
                    break
            else:
                raise RuntimeError('gslam services missing')

            if not call_bool(probe, probe.cli_gs_enable, True):
                raise RuntimeError('gs enable failed')

            # KF before valid reference: should be dropped (no crash).
            nil = UUID()
            probe.pub_kf.publish(make_kf(probe, nil, 1, 1, 0))
            spin_for(0.5)

            # Enable LS (may not produce valid ref without sensors — publish
            # synthetic KFs tagged with whatever session LS published; if still
            # invalid, admission must drop).
            if probe.cli_ls_enable.wait_for_service(timeout_sec=5.0):
                call_bool(probe, probe.cli_ls_enable, True)

            # Publish KF matching current session but possibly invalid epoch —
            # still must not crash GS.
            probe.pub_kf.publish(
                make_kf(probe, probe.last_ref.session_id, max(epoch_a0, 1), 1, 0))
            spin_for(0.5)

            # Local reset — with Attitude_update=true and no IMU, stays invalid.
            before_reset = probe.ref_n
            if not call_trig(probe, probe.cli_ls_reset):
                raise RuntimeError('ls reset failed')
            spin_for(0.5)
            if not probe.wait_ref(
                5.0,
                lambda m: (not m.valid) or probe.ref_n > before_reset,
                _spin):
                raise RuntimeError('no EstimatorReference activity after LS reset')

            # Global reset + workers still alive
            if not call_trig(probe, probe.cli_gs_reset):
                raise RuntimeError('gs reset failed')

            # Second Local SLAM instance (distinct node name) → new session.
            ls2_log = os.path.join(td, 'ls2_launch.log')
            cmd2 = [
                'ros2', 'launch', 'localslam', 'localslam.launch.py',
                f'params_file:={ls_params}',
                'node_name:=EKFslam_g3b',
                'container_name:=localslam_container_g3b',
            ] + _LS_ISOLATED_SENSORS
            with open(ls2_log, 'w', encoding='utf-8') as logf:
                p_ls2 = subprocess.Popen(
                    cmd2, stdout=logf, stderr=subprocess.STDOUT, env=env,
                    preexec_fn=os.setsid, cwd='/tmp')
            procs.append(p_ls2)
            spin_for(3.0)
            if p_ls2.poll() is not None:
                raise RuntimeError(
                    f'second Local SLAM launch exited early code={p_ls2.returncode}')
            # Use topic echo (same domain) — avoids mid-run subscription discovery issues.
            session_b = None
            t_wait = time.time()
            while time.time() - t_wait < 20.0:
                try:
                    echo = subprocess.run(
                        [
                            'ros2', 'topic', 'echo',
                            '/EKFslam_g3b/estimator_reference',
                            '--once',
                        ],
                        env=env, capture_output=True, text=True, timeout=5)
                except subprocess.TimeoutExpired:
                    continue
                if echo.returncode == 0 and 'session_id' in echo.stdout:
                    # Parse uuid bytes from YAML-ish echo output.
                    vals = []
                    for line in echo.stdout.splitlines():
                        s = line.strip()
                        if s.startswith('- ') and s[2:].isdigit():
                            vals.append(int(s[2:]))
                    if len(vals) >= 16:
                        session_b = tuple(vals[:16])
                        if session_b != session_a:
                            break
                        session_b = None
                _spin()
            if session_b is None:
                raise RuntimeError(
                    f'expected new session_id from EKFslam_g3b; session_a={session_a}')
            # Tear down first instance (recreation path).
            kill_proc(p_ls, force=True)
            procs.remove(p_ls)
            spin_for(1.0)

            # Delayed old-session KF must not be adopted (no crash / no correction).
            old = UUID()
            old.uuid = list(session_a)
            probe.pub_kf.publish(make_kf(probe, old, 1, 99, 0))
            spin_for(0.5)

            # Global restart
            kill_proc(p_gs)
            procs.remove(p_gs)
            spin_for(0.5)
            p_gs2 = launch_pkg('globalslam', 'globalslam.launch.py', env)
            procs.append(p_gs2)
            spin_for(2.0)
            if not probe.cli_gs_enable.wait_for_service(timeout_sec=20.0):
                raise RuntimeError('gs services missing after restart')
            call_bool(probe, probe.cli_gs_enable, True)
            call_trig(probe, probe.cli_gs_reset)

            if probe.ls_n != 0 or probe.ctrl_c_n != 0:
                raise RuntimeError(
                    f'corrections leaked ls={probe.ls_n} ctrl_c={probe.ctrl_c_n}')

            # Shutdown both workers present
            for p in list(procs):
                kill_proc(p)
                procs.remove(p)

            print(
                f'PASS G3 identity smoke: session_a≠session_b '
                f'({session_a[:4]}…→{session_b[:4]}…) '
                f'ref_msgs={probe.ref_n} ls_outbound={probe.ls_n} '
                f'elapsed={time.time() - t0:.1f}s')
            return 0
    except Exception as exc:
        print(f'FAIL G3 smoke: {exc}', file=sys.stderr)
        traceback.print_exc()
        return 1
    finally:
        for p in procs:
            kill_proc(p)
        execu.remove_node(probe)
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
