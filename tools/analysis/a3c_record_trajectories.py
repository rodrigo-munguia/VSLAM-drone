#!/usr/bin/env python3
"""A3c diagnostic recorder: synchronized GT / Local / Global trajectories + loop commits.

Writes NPZ + CSV under an output directory. Stops when /tmp/a3c_recorder_stop exists
or after --max-sec. Does not modify SLAM behavior.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import threading
import time
from pathlib import Path as FsPath

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from interfaces.msg import Att
from interfaces.msg import GlobalLoopCommitted
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def _stamp_sec(stamp) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def _quat_to_rpy(x, y, z, w):
    """ROS/Webots RPY (rad)."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def _rpy_to_internal(roll, pitch, yaw):
    """Match LocalSLAM / legacy Att internal convention (pitch/yaw negated)."""
    return roll, -pitch, -yaw


class A3cRecorder(Node):
    def __init__(self, out_dir: FsPath):
        super().__init__('a3c_recorder')
        self.out_dir = out_dir
        self.lock = threading.Lock()
        self.gt = []  # (t, x, y, z)
        self.odom = []  # (t, x, y, z)
        # A3.1 extensions (observational only; GT never enters estimator)
        self.gt_imu = []  # raw /imu → internal RPY + ang vel
        self.ahrs = []  # Attitude_topic (noisy AHRS fed to LocalSLAM)
        self.odom_full = []  # odom pose+twist+internal att
        self.body_vel = []  # body linear velocity twist
        self.global_path_snaps = []  # list of (t_recv, map_revision_hint, Nx3)
        self.last_global_path = None
        self.last_global_path_txyz = None  # Nx4 (source stamp, x, y, z) when available
        self.last_landmarks = None  # Nx3
        self.commits = []  # dicts
        self.pending_pre_path = None
        self._last_path_after_commit_armed = False
        self._last_lm_after_commit_armed = False

        qos_best = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        qos_path = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.create_subscription(PointStamped, '/GT_topic', self._on_gt, qos_rel)
        self.create_subscription(Odometry, '/odometry', self._on_odom, qos_best)
        self.create_subscription(Odometry, '/odometry', self._on_odom, qos_rel)
        self.create_subscription(NavPath, '/gslam/global_keyframes', self._on_gpath, qos_path)
        self.create_subscription(
            PointCloud2, '/gslam/global_landmarks', self._on_landmarks, qos_path
        )
        self.create_subscription(
            GlobalLoopCommitted, '/gslam/global_loop_committed', self._on_commit, qos_path
        )
        # GT attitude: raw Webots IMU (before bridge Att noise)
        self.create_subscription(Imu, '/imu', self._on_gt_imu, qos_rel)
        self.create_subscription(Imu, '/imu', self._on_gt_imu, qos_best)
        # AHRS measurement actually consumed by LocalSLAM (legacy Att)
        self.create_subscription(Att, '/Attitude_topic', self._on_ahrs, qos_rel)
        self.create_subscription(
            TwistWithCovarianceStamped, '/body_velocity', self._on_body_vel, qos_rel
        )
        self.create_subscription(
            TwistWithCovarianceStamped, '/body_velocity', self._on_body_vel, qos_best
        )
        self.get_logger().info(f'A3c recorder writing to {out_dir} (A3.1 att/vel enabled)')

    def _on_gt(self, msg: PointStamped):
        t = _stamp_sec(msg.header.stamp)
        with self.lock:
            self.gt.append((t, msg.point.x, msg.point.y, msg.point.z))

    def _on_gt_imu(self, msg: Imu):
        t = _stamp_sec(msg.header.stamp)
        q = msg.orientation
        roll, pitch, yaw = _quat_to_rpy(q.x, q.y, q.z, q.w)
        ir, ip, iy = _rpy_to_internal(roll, pitch, yaw)
        w = msg.angular_velocity
        with self.lock:
            self.gt_imu.append(
                (t, ir, ip, iy, roll, pitch, yaw, w.x, w.y, w.z)
            )

    def _on_ahrs(self, msg: Att):
        # Att.time is ms; prefer header-less message time field when stamp absent
        t = float(msg.time) * 1e-3 if msg.time else time.time()
        with self.lock:
            self.ahrs.append((t, float(msg.roll), float(msg.pitch), float(msg.yaw), int(msg.time)))

    def _on_body_vel(self, msg: TwistWithCovarianceStamped):
        t = _stamp_sec(msg.header.stamp)
        v = msg.twist.twist.linear
        with self.lock:
            self.body_vel.append((t, v.x, v.y, v.z))

    def _on_odom(self, msg: Odometry):
        t = _stamp_sec(msg.header.stamp)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll, pitch, yaw = _quat_to_rpy(q.x, q.y, q.z, q.w)
        ir, ip, iy = _rpy_to_internal(roll, pitch, yaw)
        tw = msg.twist.twist
        with self.lock:
            self.odom.append((t, p.x, p.y, p.z))
            self.odom_full.append(
                (
                    t,
                    p.x,
                    p.y,
                    p.z,
                    ir,
                    ip,
                    iy,
                    tw.linear.x,
                    tw.linear.y,
                    tw.linear.z,
                    tw.angular.x,
                    tw.angular.y,
                    tw.angular.z,
                )
            )

    def _path_xyz(self, path: NavPath) -> np.ndarray:
        if not path.poses:
            return np.zeros((0, 3), dtype=np.float64)
        return np.array(
            [[ps.pose.position.x, ps.pose.position.y, ps.pose.position.z] for ps in path.poses],
            dtype=np.float64,
        )

    def _path_txyz(self, path: NavPath) -> np.ndarray:
        """Per-pose KF source stamps (G4 Path) when present; else path header."""
        if not path.poses:
            return np.zeros((0, 4), dtype=np.float64)
        hdr_t = _stamp_sec(path.header.stamp) if path.header.stamp.sec else time.time()
        rows = []
        for ps in path.poses:
            if ps.header.stamp.sec or ps.header.stamp.nanosec:
                t = _stamp_sec(ps.header.stamp)
            else:
                t = hdr_t
            rows.append(
                [t, ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
            )
        return np.asarray(rows, dtype=np.float64)

    def _on_gpath(self, msg: NavPath):
        t = _stamp_sec(msg.header.stamp) if msg.header.stamp.sec else time.time()
        xyz = self._path_xyz(msg)
        txyz = self._path_txyz(msg)
        with self.lock:
            self.last_global_path = xyz
            self.last_global_path_txyz = txyz
            if self._last_path_after_commit_armed and self.commits:
                self.commits[-1]['post_path'] = xyz.copy()
                self.commits[-1]['post_t'] = t
                self._last_path_after_commit_armed = False
            # Keep sparse snapshots (~every 2 s wall) via length change
            if (
                not self.global_path_snaps
                or abs(t - self.global_path_snaps[-1][0]) > 2.0
                or xyz.shape[0] != self.global_path_snaps[-1][2].shape[0]
            ):
                self.global_path_snaps.append((t, len(self.commits), xyz.copy()))

    def _on_landmarks(self, msg: PointCloud2):
        try:
            pts = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
            xyz = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float64) if pts else np.zeros((0, 3))
        except Exception:
            xyz = np.zeros((0, 3))
        with self.lock:
            self.last_landmarks = xyz
            if self._last_lm_after_commit_armed and self.commits:
                self.commits[-1]['post_landmarks'] = xyz.copy()
                self._last_lm_after_commit_armed = False

    def _on_commit(self, msg: GlobalLoopCommitted):
        t = time.time()
        with self.lock:
            pre = None if self.last_global_path is None else self.last_global_path.copy()
            pre_lm = None if self.last_landmarks is None else self.last_landmarks.copy()
            rec = {
                'wall_t': t,
                'source': msg.source,
                'map_revision': int(msg.map_revision),
                'keyframe_id': int(msg.keyframe_id),
                'cam': (
                    float(msg.committed_camera_x),
                    float(msg.committed_camera_y),
                    float(msg.committed_camera_z),
                ),
                'match_ba_refine_ran': bool(msg.match_ba_refine_ran),
                'pre_path': pre,
                'post_path': None,
                'post_t': None,
                'pre_landmarks': pre_lm,
                'post_landmarks': None,
            }
            self.commits.append(rec)
            self._last_path_after_commit_armed = True
            self._last_lm_after_commit_armed = True
            self.get_logger().info(
                f'commit rev={rec["map_revision"]} src={rec["source"]} '
                f'cam={rec["cam"]} pre_n={0 if pre is None else len(pre)} '
                f'pre_lm={0 if pre_lm is None else len(pre_lm)}'
            )

    def flush(self) -> None:
        self.out_dir.mkdir(parents=True, exist_ok=True)
        with self.lock:
            gt = np.array(self.gt, dtype=np.float64) if self.gt else np.zeros((0, 4))
            odom = np.array(self.odom, dtype=np.float64) if self.odom else np.zeros((0, 4))
            gt_imu = (
                np.array(self.gt_imu, dtype=np.float64) if self.gt_imu else np.zeros((0, 10))
            )
            ahrs = np.array(self.ahrs, dtype=np.float64) if self.ahrs else np.zeros((0, 5))
            odom_full = (
                np.array(self.odom_full, dtype=np.float64)
                if self.odom_full
                else np.zeros((0, 13))
            )
            body_vel = (
                np.array(self.body_vel, dtype=np.float64)
                if self.body_vel
                else np.zeros((0, 4))
            )
            commits = list(self.commits)
            snaps = list(self.global_path_snaps)
            last = None if self.last_global_path is None else self.last_global_path.copy()
            last_txyz = (
                None
                if self.last_global_path_txyz is None
                else self.last_global_path_txyz.copy()
            )
            last_lm = (
                None if self.last_landmarks is None else self.last_landmarks.copy()
            )

        def _dedupe(arr):
            if len(arr) == 0:
                return arr
            keys = np.round(arr[:, 0] * 1000.0).astype(np.int64)
            _, idx = np.unique(keys, return_index=True)
            return arr[np.sort(idx)]

        gt = _dedupe(gt)
        odom = _dedupe(odom)

        np.savez_compressed(
            self.out_dir / 'trajectories.npz',
            gt=gt,
            odom=odom,
            final_global_path=last if last is not None else np.zeros((0, 3)),
            final_global_path_txyz=(
                last_txyz if last_txyz is not None else np.zeros((0, 4))
            ),
            n_commits=np.array([len(commits)]),
        )
        with (self.out_dir / 'gt.csv').open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'z'])
            w.writerows(gt.tolist())
        with (self.out_dir / 'odom.csv').open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'z'])
            w.writerows(odom.tolist())
        # A3.1: GT IMU attitude (internal + ROS RPY) and angular velocity
        with (self.out_dir / 'gt_att.csv').open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(
                [
                    't',
                    'roll_i',
                    'pitch_i',
                    'yaw_i',
                    'roll_ros',
                    'pitch_ros',
                    'yaw_ros',
                    'wx',
                    'wy',
                    'wz',
                ]
            )
            if len(gt_imu):
                w.writerows(gt_imu.tolist())
        with (self.out_dir / 'ahrs.csv').open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'roll', 'pitch', 'yaw', 'time_ms'])
            if len(ahrs):
                w.writerows(ahrs.tolist())
        with (self.out_dir / 'odom_full.csv').open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(
                [
                    't',
                    'x',
                    'y',
                    'z',
                    'roll_i',
                    'pitch_i',
                    'yaw_i',
                    'vx',
                    'vy',
                    'vz',
                    'wx',
                    'wy',
                    'wz',
                ]
            )
            if len(odom_full):
                w.writerows(odom_full.tolist())
        with (self.out_dir / 'body_vel.csv').open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'vx', 'vy', 'vz'])
            if len(body_vel):
                w.writerows(body_vel.tolist())
        # Timestamped Global KF path (public body origin; G4 per-pose stamps).
        with (self.out_dir / 'global_kf.csv').open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'z'])
            if last_txyz is not None and len(last_txyz):
                w.writerows(last_txyz.tolist())

        # Commit correction stats + landmark snapshots
        rows = []
        for i, c in enumerate(commits):
            pre, post = c.get('pre_path'), c.get('post_path')
            mean_d = rms_d = max_d = n_overlap = float('nan')
            if pre is not None and post is not None and len(pre) and len(post):
                n = min(len(pre), len(post))
                d = np.linalg.norm(post[:n] - pre[:n], axis=1)
                n_overlap = n
                mean_d = float(d.mean())
                rms_d = float(np.sqrt((d * d).mean()))
                max_d = float(d.max())
            pre_lm = c.get('pre_landmarks')
            post_lm = c.get('post_landmarks')
            lm_mean = lm_rms = lm_max = n_lm = float('nan')
            z_pre_med = z_post_med = float('nan')
            if pre_lm is not None and post_lm is not None and len(pre_lm) and len(post_lm):
                n_lm = min(len(pre_lm), len(post_lm))
                # nearest-neighbor displacement (IDs not in PointCloud2)
                # sample up to 2000 for cost
                step = max(1, n_lm // 2000)
                a = pre_lm[:n_lm:step]
                b = post_lm[:n_lm:step]
                # align by index order (publication order); imperfect without IDs
                m = min(len(a), len(b))
                dd = np.linalg.norm(b[:m] - a[:m], axis=1) if m else np.zeros(0)
                if len(dd):
                    lm_mean = float(dd.mean())
                    lm_rms = float(np.sqrt((dd * dd).mean()))
                    lm_max = float(dd.max())
                z_pre_med = float(np.median(pre_lm[:, 2]))
                z_post_med = float(np.median(post_lm[:, 2]))
            elif pre_lm is not None and len(pre_lm):
                z_pre_med = float(np.median(pre_lm[:, 2]))
            if post_lm is not None and len(post_lm):
                z_post_med = float(np.median(post_lm[:, 2]))
            if pre_lm is not None:
                np.save(self.out_dir / f'commit{i}_landmarks_pre.npy', pre_lm)
            if post_lm is not None:
                np.save(self.out_dir / f'commit{i}_landmarks_post.npy', post_lm)
            if pre is not None:
                np.save(self.out_dir / f'commit{i}_path_pre.npy', pre)
            if post is not None:
                np.save(self.out_dir / f'commit{i}_path_post.npy', post)
            rows.append(
                {
                    'idx': i,
                    'source': c['source'],
                    'map_revision': c['map_revision'],
                    'keyframe_id': c['keyframe_id'],
                    'cam_x': c['cam'][0],
                    'cam_y': c['cam'][1],
                    'cam_z': c['cam'][2],
                    'match_ba_refine_ran': int(c['match_ba_refine_ran']),
                    'n_kf_overlap': n_overlap,
                    'mean_kf_delta_m': mean_d,
                    'rms_kf_delta_m': rms_d,
                    'max_kf_delta_m': max_d,
                    'n_lm_overlap': n_lm,
                    'mean_lm_delta_m': lm_mean,
                    'rms_lm_delta_m': lm_rms,
                    'max_lm_delta_m': lm_max,
                    'z_pre_median': z_pre_med,
                    'z_post_median': z_post_med,
                    'n_lm_pre': 0 if pre_lm is None else len(pre_lm),
                    'n_lm_post': 0 if post_lm is None else len(post_lm),
                }
            )
        with (self.out_dir / 'commits.csv').open('w', newline='') as f:
            fields = list(rows[0].keys()) if rows else [
                'idx', 'source', 'map_revision', 'keyframe_id',
                'cam_x', 'cam_y', 'cam_z', 'match_ba_refine_ran',
                'n_kf_overlap', 'mean_kf_delta_m', 'rms_kf_delta_m', 'max_kf_delta_m',
                'n_lm_overlap', 'mean_lm_delta_m', 'rms_lm_delta_m', 'max_lm_delta_m',
                'z_pre_median', 'z_post_median', 'n_lm_pre', 'n_lm_post',
            ]
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            w.writerows(rows)

        # Sparse global snapshots meta
        with (self.out_dir / 'global_snaps_meta.csv').open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'n_commits_so_far', 'n_kf'])
            for t, nc, xyz in snaps:
                w.writerow([t, nc, xyz.shape[0]])
        if snaps:
            # save last few full paths for offline ATE
            for name, idx in [('global_path_final', -1), ('global_path_mid', len(snaps) // 2)]:
                np.save(self.out_dir / f'{name}.npy', snaps[idx][2])
        if last_lm is not None and len(last_lm):
            np.save(self.out_dir / 'global_landmarks_final.npy', last_lm)

        summary = (
            f'gt={len(gt)} odom={len(odom)} gt_att={len(gt_imu)} ahrs={len(ahrs)} '
            f'odom_full={len(odom_full)} body_vel={len(body_vel)} commits={len(commits)} '
            f'global_snaps={len(snaps)} final_gkf={0 if last is None else len(last)}\n'
        )
        (self.out_dir / 'recorder_summary.txt').write_text(summary)
        print(summary, end='')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--max-sec', type=float, default=3600.0)
    ap.add_argument('--stop-file', default='/tmp/a3c_recorder_stop')
    args = ap.parse_args()
    out = FsPath(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    stop = FsPath(args.stop_file)
    if stop.exists():
        stop.unlink()

    rclpy.init()
    node = A3cRecorder(out)
    t0 = time.time()
    last_flush = t0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.time()
            if stop.exists() or (now - t0) > args.max_sec:
                break
            if (now - last_flush) >= 60.0:
                node.flush()
                last_flush = now
    finally:
        node.flush()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
