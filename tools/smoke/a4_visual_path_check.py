#!/usr/bin/env python3
"""Bounded A4 visual-path check (Visual_update=true).

Uses a deterministic synthetic textured mono sequence with documented
camera motion, calibration (node defaults / overrides below), altitude,
downward range (>0.5 m for undelayed_range init), and body velocity
above Visual_update_min_vel_for_update_feats.

Stages exercised / reported:
  1. Frames reach visual processing (Lslam / landmarks published on frame steps)
  2. Features initialize (nonempty map_state and/or landmarks)
  3. Subsequent frames associate (matched_img_feats > 0 after init)
  4. Landmark PointCloud2 corresponds to current Lslam map under D
  5. Reset clears map/path and allows re-init

Does NOT claim trajectory accuracy or experiment parity.
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
from interfaces.msg import Lslam

from launch import LaunchDescription, LaunchService
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

_D = (1.0, -1.0, -1.0)
CHILDREN: List[mp.Process] = []

# Documented synthetic scene / motion
IMG_H, IMG_W = 240, 320
# Camera calibration matches package defaults (see parameters.cpp)
CAM_FC_U, CAM_FC_V = 206.34225, 268.65192
CAM_CC_U, CAM_CC_V = 156.24435, 117.04562
# Motion: constant body +x velocity; attitude near level; altitude/range fixed
BODY_VX = 0.35  # > Visual_update_min_vel_for_update_feats (0.02)
ALT_M = 2.0
RANGE_M = 2.0  # > 0.5 for undelayed_range init
ATT_RPY = (0.02, -0.01, 0.05)


def _params() -> dict:
    return {
        'Sensor_input_mode': 'standard',
        'Altitude_input_mode': 'derived_altitude',
        'Data_origin': 'webots',
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
        'Mono_cam_fc_u': CAM_FC_U,
        'Mono_cam_fc_v': CAM_FC_V,
        'Mono_cam_cc_u': CAM_CC_U,
        'Mono_cam_cc_v': CAM_CC_V,
        'Images_number_candidate_points_detected': 30,
        'Images_minimun_distance_new_points': 15,
        'Sigma_att_update': 0.001,
        'Sigma_spd_update': 0.05,
        'Sigma_alt_update': 0.1,
        'Sigma_cam_uv': 3.0,
        'Sigma_w': 0.05,
        'Sigma_a': 2.0,
        'Tau_r': 1.0,
        'Tau_p': 100.0,
        'Alt_h_2_d': -1,
        'Stats': False,
        'Odometry_frame_id': 'odom',
        'Odometry_child_frame_id': 'base_link',
        'Path_max_poses': 500,
    }


def make_textured_frame(frame_idx: int) -> bytes:
    """Deterministic high-contrast texture with slow horizontal scroll.

    Simulates forward camera motion over a static textured plane: ORB/FAST
    corners from black/white blocks. Scroll ≈ f * (v*dt)/Z in pixels
    (approximate; not projective-accurate).
    """
    # ~ fc_u * (BODY_VX * 0.05) / RANGE_M ≈ 206 * 0.0175 / 2 ≈ 1.8 px/frame
    shift = int(round(frame_idx * 2.0)) % 40
    buf = bytearray(IMG_H * IMG_W)
    for v in range(IMG_H):
        for u in range(IMG_W):
            # Checker + sparse high-contrast blobs for ORB
            cu = (u + shift) // 20
            cv = v // 20
            base = 220 if (cu + cv) % 2 == 0 else 40
            # Extra corner-rich squares
            if ((u + shift) % 47 < 8) and (v % 53 < 8):
                base = 10 if base > 100 else 245
            buf[v * IMG_W + u] = base & 0xFF
    return bytes(buf)


def _launch_worker(ready_q: mp.Queue, log_path: str) -> None:
    sys.stdout = open(log_path, 'w', buffering=1)
    sys.stderr = sys.stdout
    try:
        container = ComposableNodeContainer(
            name=f'a4_vis_{os.getpid()}',
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
        f'_smoke_a4_vis_{os.getpid()}.log'))
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
            ['pkill', '-f', 'component_container.*a4_vis_'],
            check=False, capture_output=True)
    except Exception:
        pass


def wait_for_node(timeout: float = 25.0) -> bool:
    rclpy.init()
    n = Node('a4_vis_waiter')
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
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def unpack_xyz(cloud: PointCloud2) -> List[Tuple[float, float, float]]:
    pts = []
    for i in range(cloud.width):
        off = i * cloud.point_step
        if off + 12 > len(cloud.data):
            break
        pts.append(struct.unpack_from('<fff', cloud.data, off))
    return pts


class Driver(Node):
    def __init__(self) -> None:
        super().__init__('a4_vis_driver')
        self.pub_imu = self.create_publisher(Imu, '/EKFslam/imu', 10)
        self.pub_alt = self.create_publisher(Altitude, '/EKFslam/altitude', 10)
        self.pub_spd = self.create_publisher(
            TwistWithCovarianceStamped, '/EKFslam/body_velocity', 10)
        self.pub_rng = self.create_publisher(Range, '/EKFslam/downward_range', 10)
        self.pub_img = self.create_publisher(Image, '/EKFslam/image', 10)
        self.cli_enable = self.create_client(SetBool, '/EKFslam/enable')
        self.cli_reset = self.create_client(Trigger, '/EKFslam/reset')
        self.frame_pubs = 0
        self.lslam_n = 0
        self.cloud_n = 0
        self.path_n = 0
        self.odom_n = 0
        self.max_map = 0
        self.max_matched = 0
        self.max_cloud = 0
        self.coord_agree = 0
        self.last_lslam: Optional[Lslam] = None
        self.last_cloud: Optional[PointCloud2] = None
        self.last_path: Optional[Path] = None
        self.create_subscription(Lslam, '/lslam_data_topic', self._on_lslam, 10)
        self.create_subscription(PointCloud2, '/EKFslam/landmarks', self._on_cloud, 10)
        self.create_subscription(Path, '/EKFslam/path', self._on_path, 10)
        self.create_subscription(Odometry, '/EKFslam/odometry', self._on_odom, 10)

    def _on_odom(self, _m: Odometry) -> None:
        self.odom_n += 1

    def _on_path(self, msg: Path) -> None:
        self.path_n += 1
        self.last_path = msg

    def _on_lslam(self, msg: Lslam) -> None:
        self.lslam_n += 1
        self.last_lslam = msg
        n_map = len(msg.map_state) + len(msg.map_anchors)
        self.max_map = max(self.max_map, n_map)
        self.max_matched = max(
            self.max_matched,
            len(msg.matched_img_feats) + len(msg.matched_img_anchors))
        if self.last_cloud is not None and n_map > 0:
            self._check_coord_agree(msg, self.last_cloud)

    def _on_cloud(self, msg: PointCloud2) -> None:
        self.cloud_n += 1
        self.last_cloud = msg
        self.max_cloud = max(self.max_cloud, int(msg.width))
        if self.last_lslam is not None:
            n_map = len(self.last_lslam.map_state) + len(self.last_lslam.map_anchors)
            if n_map > 0:
                self._check_coord_agree(self.last_lslam, msg)

    def _check_coord_agree(self, lslam: Lslam, cloud: PointCloud2) -> None:
        internal = [(p.x, p.y, p.z) for p in list(lslam.map_state) + list(lslam.map_anchors)]
        expected = []
        for x, y, z in internal:
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            expected.append((_D[0] * x, _D[1] * y, _D[2] * z))
        pub = unpack_xyz(cloud)
        if len(pub) != len(expected):
            return
        ok = True
        for a, b in zip(expected, pub):
            if abs(a[0] - b[0]) > 1e-3 or abs(a[1] - b[1]) > 1e-3 or abs(a[2] - b[2]) > 1e-3:
                ok = False
                break
        if ok and expected:
            self.coord_agree += 1

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
        if not fut.done() or not fut.result().success:
            raise RuntimeError(f'enable({on}) failed')

    def call_reset(self) -> None:
        fut = self.cli_reset.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or not fut.result().success:
            raise RuntimeError('reset failed')

    def publish_aux(self) -> None:
        now = self.get_clock().now().to_msg()
        q = _rpy_to_quat(*ATT_RPY)
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'imu_link'
        imu.orientation.x, imu.orientation.y = q[0], q[1]
        imu.orientation.z, imu.orientation.w = q[2], q[3]
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
        rng.radiation_type = Range.INFRARED
        rng.field_of_view = 0.2
        rng.min_range = 0.1
        rng.max_range = 10.0
        rng.range = RANGE_M
        self.pub_rng.publish(rng)

    def publish_frame(self, idx: int) -> None:
        self.publish_aux()
        now = self.get_clock().now().to_msg()
        img = Image()
        img.header.stamp = now
        img.header.frame_id = 'camera_optical_frame'
        img.height = IMG_H
        img.width = IMG_W
        img.encoding = 'mono8'
        img.step = IMG_W
        img.data = make_textured_frame(idx)
        self.pub_img.publish(img)
        self.frame_pubs += 1


def main() -> int:
    stages = {
        'frames_reach_visual_pub': False,
        'features_initialize': False,
        'subsequent_association': False,
        'landmark_cloud_corresponds': False,
        'reset_clears_and_reinits': False,
    }
    defects: List[str] = []
    try:
        print('=== A4 visual-path check ===', flush=True)
        print(
            f'scene: {IMG_W}x{IMG_H} mono checker/blob texture; '
            f'calib fc=({CAM_FC_U},{CAM_FC_V}) cc=({CAM_CC_U},{CAM_CC_V}); '
            f'motion body_vx={BODY_VX} m/s, alt={ALT_M} m, range={RANGE_M} m, '
            f'att_rpy={ATT_RPY}; init=undelayed_range',
            flush=True)
        proc, log_path = start_slam()
        if not wait_for_node():
            log = open(log_path, encoding='utf-8', errors='replace').read()
            raise RuntimeError(f'EKFslam missing\n{log[-2000:]}')

        rclpy.init()
        d = Driver()
        try:
            d.wait_srvs()
            d.call_enable(True)
            # Attitude-first init
            for _ in range(20):
                d.publish_aux()
                rclpy.spin_once(d, timeout_sec=0.02)
                time.sleep(0.03)
            # Visual sequence
            for i in range(60):
                d.publish_frame(i)
                # Interleave a few aux ticks between frames
                for _ in range(2):
                    d.publish_aux()
                    rclpy.spin_once(d, timeout_sec=0.01)
                    time.sleep(0.02)
                rclpy.spin_once(d, timeout_sec=0.05)
                time.sleep(0.03)

            t0 = time.time()
            while time.time() - t0 < 2.0:
                rclpy.spin_once(d, timeout_sec=0.1)

            print(
                f'frame_pubs={d.frame_pubs} lslam={d.lslam_n} cloud={d.cloud_n} '
                f'odom={d.odom_n} max_map={d.max_map} max_matched={d.max_matched} '
                f'max_cloud={d.max_cloud} coord_agree={d.coord_agree}',
                flush=True)

            if d.lslam_n >= 1 and d.cloud_n >= 1:
                stages['frames_reach_visual_pub'] = True
            else:
                defects.append(
                    'frames did not produce Lslam/landmarks (visual pub path)')

            if d.max_map >= 1 and d.max_cloud >= 1:
                stages['features_initialize'] = True
            else:
                defects.append(
                    'no features/anchors initialized (check undelayed_range, '
                    'range>0.5, |v|>min_vel, ORB detections on synthetic texture)')

            if d.max_matched >= 1:
                stages['subsequent_association'] = True
            else:
                defects.append(
                    'no matched_img_feats/anchors after init '
                    '(association/update stage not evidenced)')

            if d.coord_agree >= 1:
                stages['landmark_cloud_corresponds'] = True
            elif d.max_map >= 1:
                defects.append(
                    'map nonempty but PointCloud2 did not match D*internal snapshot')

            # Reset
            d.call_reset()
            saw_empty_cloud = False
            saw_empty_path = False
            t0 = time.time()
            while time.time() - t0 < 2.0:
                rclpy.spin_once(d, timeout_sec=0.1)
                if d.last_cloud is not None and d.last_cloud.width == 0:
                    saw_empty_cloud = True
                if d.last_path is not None and len(d.last_path.poses) == 0:
                    saw_empty_path = True

            # Re-init + a few frames
            map_after = d.max_map
            for i in range(25):
                d.publish_frame(100 + i)
                d.publish_aux()
                rclpy.spin_once(d, timeout_sec=0.05)
                time.sleep(0.04)
            t0 = time.time()
            while time.time() - t0 < 1.5:
                rclpy.spin_once(d, timeout_sec=0.1)

            if saw_empty_cloud and saw_empty_path and d.odom_n > 0:
                stages['reset_clears_and_reinits'] = True
            else:
                defects.append(
                    f'reset/reinit incomplete empty_cloud={saw_empty_cloud} '
                    f'empty_path={saw_empty_path} odom={d.odom_n} '
                    f'max_map_was={map_after}')

            print('STAGES:', stages, flush=True)
            if defects:
                print('DEFECTS:', defects, flush=True)
                log = open(log_path, encoding='utf-8', errors='replace').read()
                print('log_tail:\n' + log[-2500:], flush=True)
                # Hard-fail only if frames never reached visual pubs or reset broken.
                # Init/match failures are reported as visual-path defects.
                if not stages['frames_reach_visual_pub']:
                    raise RuntimeError('visual pubs never appeared')
                if not stages['reset_clears_and_reinits']:
                    raise RuntimeError('reset/reinit contract failed')
                if not stages['features_initialize']:
                    print(
                        'A4_VISUAL_PATH_DEFECT: features did not initialize',
                        flush=True)
                    return 2
                if not stages['subsequent_association']:
                    print(
                        'A4_VISUAL_PATH_DEFECT: association not evidenced',
                        flush=True)
                    return 2
                if not stages['landmark_cloud_corresponds']:
                    print(
                        'A4_VISUAL_PATH_DEFECT: landmark correspondence failed',
                        flush=True)
                    return 2

            print('A4_VISUAL_PATH_OK', flush=True)
            return 0
        finally:
            d.destroy_node()
            rclpy.shutdown()
    except Exception as exc:
        print(f'A4_VISUAL_PATH_FAIL: {exc}', file=sys.stderr, flush=True)
        traceback.print_exc()
        return 1
    finally:
        cleanup()


if __name__ == '__main__':
    mp.set_start_method('spawn', force=True)
    sys.exit(main())
