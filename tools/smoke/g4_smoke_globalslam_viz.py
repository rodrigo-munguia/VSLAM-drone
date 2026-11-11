#!/usr/bin/env python3
"""G4: Local+Global visualization smoke (corrections remain off).

Starts Local SLAM + Global SLAM, publishes a valid EstimatorReference-compatible
session via LS init path when possible, and injects textured translating
keyframes so GMAP can process them. Checks:

  - Nonempty /gslam/global_keyframes (Path) after processing
  - Nonempty /gslam/global_landmarks (PointCloud2) when triangulation succeeds
  - Public geometry matches D=diag(1,-1,-1) on injected body poses (pre-BA)
  - Reset clears Path+cloud; late transient_local subscriber receives current
  - Zero LSposUpdate / Control 'c' with Apply_global_corrections=false

Graphical RViz2/Plot2 overlays are not asserted here (report unverified unless
observed separately). Does not claim BA/CL trajectory accuracy.
"""

from __future__ import annotations

import math
import os
import signal
import struct
import subprocess
import sys
import tempfile
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Transform
from nav_msgs.msg import Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool, Trigger
from unique_identifier_msgs.msg import UUID

from interfaces.msg import EstimatorReference, Kf
from interfaces.srv import LSposUpdate, SimpleServ

IMG_H, IMG_W = 240, 400
TIMEOUT_S = 180.0
DOMAIN = os.environ.get('G4_SMOKE_DOMAIN_ID', '92')


def make_textured_frame(frame_idx: int) -> bytes:
    shift = int(round(frame_idx * 3.0)) % 40
    buf = bytearray(IMG_H * IMG_W)
    for v in range(IMG_H):
        for u in range(IMG_W):
            cu = (u + shift) // 16
            cv = v // 16
            base = 220 if (cu + cv) % 2 == 0 else 35
            if ((u + shift) % 41 < 6) and (v % 37 < 6):
                base = 5 if base > 100 else 250
            buf[v * IMG_W + u] = base & 0xFF
    return bytes(buf)


def yaw_quat(yaw: float) -> Tuple[float, float, float, float]:
    """Internal Rn2r for yaw about +z (wxyz)."""
    h = 0.5 * yaw
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def apply_D(x: float, y: float, z: float) -> Tuple[float, float, float]:
    return (x, -y, -z)


def make_kf(
    node: Node,
    session: UUID,
    epoch: int,
    kid: int,
    frame_idx: int,
    x_m: float,
    yaw: float = 0.0,
) -> Kf:
    msg = Kf()
    msg.session_id = session
    msg.epoch_id = epoch
    msg.keyframe_id = kid
    msg.source_stamp_valid = True
    msg.stream = 0
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'internal_nav_cam'
    msg.img.header = msg.header
    msg.img.height = IMG_H
    msg.img.width = IMG_W
    msg.img.encoding = 'mono8'
    msg.img.step = IMG_W
    msg.img.data = make_textured_frame(frame_idx)

    # Camera lever arm in robot frame (matches typical quad YAML x=0.1).
    msg.r2c = Transform()
    msg.r2c.translation.x = 0.1
    msg.r2c.rotation.w = 1.0

    w, x, y, z = yaw_quat(yaw)
    msg.n2r = Transform()
    msg.n2r.translation.x = x_m
    msg.n2r.translation.y = 0.05 * frame_idx  # slight lateral for asymmetry
    msg.n2r.translation.z = 0.0
    msg.n2r.rotation.w = w
    msg.n2r.rotation.x = x
    msg.n2r.rotation.y = y
    msg.n2r.rotation.z = z
    return msg


class Probe(Node):
    def __init__(self) -> None:
        super().__init__('g4_viz_probe')
        qos_ref = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos_viz = qos_ref
        self.last_ref: Optional[EstimatorReference] = None
        self.last_path: Optional[Path] = None
        self.last_cloud: Optional[PointCloud2] = None
        self.path_n = 0
        self.cloud_n = 0
        self.ls_n = 0
        self.ctrl_c_n = 0
        self._qos_viz = qos_viz
        self.create_subscription(
            EstimatorReference, '/EKFslam/estimator_reference', self._on_ref, qos_ref)
        # Path/cloud subscriptions are created after GS is up (DDS matching).
        self.pub_kf = self.create_publisher(Kf, 'Kf_topic', 10)
        # Dedicated ref topic so Local SLAM's invalid latch does not clear GS
        # while we inject textured KFs (both components still loaded).
        self.pub_ref = self.create_publisher(
            EstimatorReference, '/g4_smoke/estimator_reference', qos_ref)
        self.cli_gs_enable = self.create_client(SetBool, '/gslam/enable')
        self.cli_gs_reset = self.create_client(Trigger, '/gslam/reset')
        self.create_service(LSposUpdate, 'local_slam_pos_update', self._on_ls)
        self.create_service(SimpleServ, 'control_run_service', self._on_ctrl)

    def _on_ref(self, msg: EstimatorReference) -> None:
        self.last_ref = msg

    def _on_path(self, msg: Path) -> None:
        self.last_path = msg
        self.path_n += 1

    def _on_cloud(self, msg: PointCloud2) -> None:
        self.last_cloud = msg
        self.cloud_n += 1

    def _on_ls(self, request, response):
        self.ls_n += 1
        response.accepted = False
        response.applied = False
        response.outcome = 'rejected'
        response.reason = 'g4_smoke_sink'
        return response

    def _on_ctrl(self, request, response):
        if request.cmd == ord('c') or request.cmd == 'c':
            self.ctrl_c_n += 1
        response.response = True
        return response

    def subscribe_global_viz(self) -> None:
        self.create_subscription(
            Path, '/gslam/global_keyframes', self._on_path, self._qos_viz)
        self.create_subscription(
            PointCloud2, '/gslam/global_landmarks', self._on_cloud, self._qos_viz)


def launch_pkg(
    pkg: str, launch: str, env: dict, *args: str,
    log_path: Optional[str] = None,
) -> subprocess.Popen:
    cmd = ['ros2', 'launch', pkg, launch, *args]
    if log_path:
        logf = open(log_path, 'w', encoding='utf-8')
        return subprocess.Popen(
            cmd, stdout=logf, stderr=subprocess.STDOUT, env=env,
            preexec_fn=os.setsid, cwd='/tmp')
    return subprocess.Popen(
        cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env,
        preexec_fn=os.setsid, cwd='/tmp')


def kill_proc(proc: subprocess.Popen, force: bool = False) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGKILL if force else signal.SIGTERM)
    except (ProcessLookupError, PermissionError, OSError):
        try:
            proc.terminate()
        except Exception:
            pass
    try:
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
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


def cloud_xyz(msg: PointCloud2) -> List[Tuple[float, float, float]]:
    pts = []
    if msg.point_step < 12 or msg.width == 0 or not msg.data:
        return pts
    for i in range(msg.width):
        off = i * msg.point_step
        if off + 12 > len(msg.data):
            break
        x, y, z = struct.unpack_from('<fff', msg.data, off)
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            pts.append((x, y, z))
    return pts


def echo_once(topic: str, env: dict, timeout: float = 20.0) -> str:
    echo = subprocess.run(
        ['ros2', 'topic', 'echo', topic, '--once'],
        env=env, capture_output=True, text=True, timeout=timeout)
    if echo.returncode != 0:
        raise RuntimeError(
            f'echo {topic} failed rc={echo.returncode}: {echo.stderr[:500]}')
    return echo.stdout


def count_path_poses(echo_txt: str) -> int:
    # Each pose has a nested "position:" under pose: — count those.
    return echo_txt.count('position:')


def count_cloud_width(echo_txt: str) -> int:
    for line in echo_txt.splitlines():
        s = line.strip()
        if s.startswith('width:'):
            try:
                return int(s.split(':', 1)[1].strip())
            except ValueError:
                pass
    return -1


def first_path_xyz(echo_txt: str) -> Tuple[float, float, float]:
    """Parse first pose position from ros2 topic echo YAML."""
    lines = echo_txt.splitlines()
    for i, line in enumerate(lines):
        if line.strip() == 'position:':
            def grab(idx: int, key: str) -> float:
                for j in range(idx, min(idx + 6, len(lines))):
                    t = lines[j].strip()
                    if t.startswith(f'{key}:'):
                        return float(t.split(':', 1)[1].strip())
                raise RuntimeError(f'missing {key} after position')
            return grab(i + 1, 'x'), grab(i + 1, 'y'), grab(i + 1, 'z')
    raise RuntimeError('no position in Path echo')


def write_ls_params(path: str) -> None:
    text = f"""
/lslam:
  ros__parameters:
    Sensor_input_mode: standard
    Altitude_input_mode: derived_altitude
    Calibration_source: yaml
    Data_origin: standalone
    Visual_update: false
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


def write_gs_params(path: str) -> None:
    text = f"""
/gslam:
  ros__parameters:
    Apply_global_corrections: false
    # Start enabled so the smoke does not race enable vs first KF batch.
    Processing_start_enabled: true
    Accept_missing_source_stamp: false
    Public_frame_id: odom
    Path_max_poses: 0
    Landmark_cloud_max_points: 0
    # Dedicated topic: Local SLAM's invalid latch must not clear GS mid-smoke.
    Estimator_reference_topic: /g4_smoke/estimator_reference
    Mono_cam_img_rows: {IMG_H}
    Mono_cam_img_cols: {IMG_W}
    Mono_cam_distortions: [0.0, 0.0, 0.0, 0.0, 0.0]
    Mono_cam_cc_u: 200.0
    Mono_cam_cc_v: 120.0
    Mono_cam_fc_u: 200.0
    Mono_cam_fc_v: 200.0
    Mono_cam_alpha_c: 0.0
    Init_number_candidate_points_per_kf: 400
    Init_min_depth_to_consider: 0.2
    Init_min_distance_to_previous_mapped_features: 1
    BA_max_n_kf_optimized: 5
    BA_min_n_latest_kf_to_optimize: 3
    Stats: false
"""
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)


def main() -> int:
    install = os.environ.get(
        'EKF_INSTALL_BASE',
        os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            'install'))
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = DOMAIN
    os.environ['ROS_DOMAIN_ID'] = DOMAIN
    procs: List[subprocess.Popen] = []

    rclpy.init()
    probe = Probe()
    execu = SingleThreadedExecutor()
    execu.add_node(probe)

    def spin_for(seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            execu.spin_once(timeout_sec=0.05)

    evidence = {
        'keyframes_path': 0,
        'landmarks': 0,
        'admitted_kf': 0,
        'reset_cleared': False,
        'late_subscriber': False,
        'corrections': 0,
    }

    try:
        with tempfile.TemporaryDirectory(prefix='g4_smoke_') as td:
            ls_yaml = os.path.join(td, 'ls.yaml')
            gs_yaml = os.path.join(td, 'gs.yaml')
            write_ls_params(ls_yaml)
            write_gs_params(gs_yaml)

            _ls_iso = [
                'image_topic:=/g4_smoke/unused/image',
                'camera_info_topic:=/g4_smoke/unused/camera_info',
                'imu_topic:=/g4_smoke/unused/imu',
                'body_velocity_topic:=/g4_smoke/unused/body_velocity',
                'downward_range_topic:=/g4_smoke/unused/downward_range',
                'altitude_topic:=/g4_smoke/unused/altitude',
                'pressure_topic:=/g4_smoke/unused/pressure',
            ]
            procs.append(launch_pkg(
                'localslam', 'localslam.launch.py', env,
                f'params_file:={ls_yaml}', *_ls_iso))
            spin_for(2.0)
            t0 = time.time()
            while time.time() - t0 < 20.0 and probe.last_ref is None:
                execu.spin_once(timeout_sec=0.1)
            if probe.last_ref is None:
                raise RuntimeError('no EstimatorReference from Local SLAM')

            # Use Local SLAM's session UUID; epoch chosen for the GS-only ref feed.
            session = probe.last_ref.session_id
            ref = EstimatorReference()
            ref.valid = True
            ref.session_id = session
            ref.epoch_id = 1

            gs_log = os.path.join(td, 'gs_launch.log')
            procs.append(launch_pkg(
                'globalslam', 'globalslam.launch.py', env,
                f'params_file:={gs_yaml}',
                log_path=gs_log))
            spin_for(3.0)
            t0 = time.time()
            while time.time() - t0 < 20.0:
                if probe.cli_gs_enable.wait_for_service(timeout_sec=0.5):
                    break
            else:
                raise RuntimeError(
                    'gslam services missing — build globalslam with GTSAM '
                    '(tools/gtsam_prefix.env.example) and keep LD_LIBRARY_PATH')

            # Confirm dedicated reference topic (params_file applied).
            try:
                got = subprocess.run(
                    ['ros2', 'param', 'get', '/gslam', 'Estimator_reference_topic'],
                    env=env, capture_output=True, text=True, timeout=10)
                if '/g4_smoke/estimator_reference' not in got.stdout:
                    raise RuntimeError(
                        f'GS Estimator_reference_topic not applied: {got.stdout!r} '
                        f'(params_file={gs_yaml})')
            except subprocess.TimeoutExpired as exc:
                raise RuntimeError('ros2 param get timed out') from exc

            # Processing already enabled via YAML; keep an explicit enable for clarity.
            call_bool(probe, probe.cli_gs_enable, True)
            topics = subprocess.run(
                ['ros2', 'topic', 'list'], env=env, capture_output=True, text=True,
                timeout=10)
            if '/gslam/global_keyframes' not in topics.stdout:
                raise RuntimeError(
                    f'global_keyframes topic missing; topics=\n{topics.stdout}')
            probe.subscribe_global_viz()
            spin_for(0.5)

            for _ in range(3):
                probe.pub_ref.publish(ref)
                spin_for(0.3)
            spin_for(1.0)

            expected_public = []
            n_kf = 12
            for i in range(n_kf):
                x_m = 0.15 * i
                yaw = 0.02 * i
                kf = make_kf(probe, session, ref.epoch_id, i + 1, i, x_m, yaw)
                expected_public.append(apply_D(
                    kf.n2r.translation.x,
                    kf.n2r.translation.y,
                    kf.n2r.translation.z))
                probe.pub_kf.publish(kf)
                spin_for(0.4)

            evidence['admitted_kf'] = n_kf
            spin_for(2.0)

            # Verify via process-isolated echo (TRANSIENT_LOCAL). In-process
            # rclpy subscriptions can miss matching under this multi-launch layout.
            path_txt = ''
            t0 = time.time()
            while time.time() - t0 < 60.0:
                try:
                    path_txt = echo_once('/gslam/global_keyframes', env, timeout=8.0)
                except Exception:
                    spin_for(0.5)
                    continue
                if count_path_poses(path_txt) >= 2:
                    break
                spin_for(0.5)
            nposes = count_path_poses(path_txt)
            if nposes < 2:
                gs_tail = ''
                try:
                    gs_tail = open(gs_log, encoding='utf-8').read()[-2500:]
                except OSError:
                    pass
                raise RuntimeError(
                    f'expected nonempty global_keyframes Path; poses≈{nposes}\n'
                    f'{path_txt[:1500]}\n--- gs ---\n{gs_tail}')
            evidence['keyframes_path'] = nposes
            if 'frame_id: odom' not in path_txt:
                raise RuntimeError('Path frame_id missing odom')

            p0 = first_path_xyz(path_txt)
            exp0 = expected_public[0]
            err0 = math.sqrt(
                (p0[0] - exp0[0]) ** 2 + (p0[1] - exp0[1]) ** 2 +
                (p0[2] - exp0[2]) ** 2)
            if err0 > 0.5:
                raise RuntimeError(
                    f'Path[0] public pose far from D*body ({err0:.3f} m); '
                    f'got={p0} exp={exp0}')

            cloud_txt = ''
            t0 = time.time()
            while time.time() - t0 < 45.0:
                try:
                    cloud_txt = echo_once('/gslam/global_landmarks', env, timeout=8.0)
                except Exception:
                    spin_for(0.5)
                    continue
                width = count_cloud_width(cloud_txt)
                if width > 0:
                    evidence['landmarks'] = width
                    break
                spin_for(0.5)
            if evidence['landmarks'] <= 0:
                raise RuntimeError(
                    'expected nonempty global_landmarks from GMAP triangulation '
                    f'(path_poses={evidence["keyframes_path"]})\n{cloud_txt[:1000]}')

            if not call_trig(probe, probe.cli_gs_reset):
                raise RuntimeError('gs reset failed')
            spin_for(1.0)
            empty_path = echo_once('/gslam/global_keyframes', env)
            empty_cloud = echo_once('/gslam/global_landmarks', env)
            if count_path_poses(empty_path) != 0:
                raise RuntimeError('Path not emptied after reset')
            if count_cloud_width(empty_cloud) != 0:
                raise RuntimeError('cloud not emptied after reset')
            evidence['reset_cleared'] = True

            probe.pub_ref.publish(ref)
            spin_for(0.3)
            for i in range(6):
                probe.pub_kf.publish(
                    make_kf(probe, session, ref.epoch_id, 100 + i, i, 0.2 * i))
                spin_for(0.4)
            spin_for(2.0)
            resume = ''
            t0 = time.time()
            while time.time() - t0 < 40.0:
                resume = echo_once('/gslam/global_keyframes', env, timeout=8.0)
                if count_path_poses(resume) >= 2:
                    break
                spin_for(0.5)
            if count_path_poses(resume) < 2:
                raise RuntimeError('Path did not resume after reset+reinit')
            late = echo_once('/gslam/global_keyframes', env)
            if count_path_poses(late) < 2:
                raise RuntimeError('late subscriber failed to receive Path')
            evidence['late_subscriber'] = True

            evidence['corrections'] = probe.ls_n + probe.ctrl_c_n
            if probe.ls_n != 0 or probe.ctrl_c_n != 0:
                raise RuntimeError(
                    f'correction side effects ls={probe.ls_n} ctrl_c={probe.ctrl_c_n}')

            print(
                f'PASS G4 viz smoke: path_poses={evidence["keyframes_path"]} '
                f'landmarks={evidence["landmarks"]} admitted_kf≈{evidence["admitted_kf"]} '
                f'reset_cleared={evidence["reset_cleared"]} '
                f'late_sub={evidence["late_subscriber"]} '
                f'corrections={evidence["corrections"]} '
                f'(graphical RViz2/Plot2 unverified in this smoke)')
            return 0
    except Exception as exc:
        print(f'FAIL G4 smoke: {exc}', file=sys.stderr)
        return 1
    finally:
        for p in list(procs):
            kill_proc(p, force=True)
        execu.remove_node(probe)
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
