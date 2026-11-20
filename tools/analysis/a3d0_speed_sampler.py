#!/usr/bin/env python3
"""Sample body_velocity + odometry + GT during A3d speed smoke."""
from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class Sampler(Node):
    def __init__(self):
        super().__init__('a3d0_speed_sampler')
        self.vel = []
        self.odom = []
        self.gt = []
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        # body_velocity may be Twist or TwistStamped depending on bridge
        self.create_subscription(Odometry, '/odometry', self._odom, qos_be)
        self.create_subscription(Odometry, '/odometry', self._odom, qos_rel)
        self.create_subscription(PointStamped, '/GT_topic', self._gt, qos_rel)
        try:
            from geometry_msgs.msg import Twist

            self.create_subscription(Twist, '/body_velocity', self._twist, qos_be)
            self.create_subscription(Twist, '/body_velocity', self._twist, qos_rel)
        except Exception:
            pass
        self.create_subscription(TwistStamped, '/body_velocity', self._twist_st, qos_be)
        self.create_subscription(TwistStamped, '/body_velocity', self._twist_st, qos_rel)

    def _stamp(self, stamp) -> float:
        return float(stamp.sec) + 1e-9 * float(stamp.nanosec)

    def _odom(self, msg: Odometry):
        t = self._stamp(msg.header.stamp)
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.odom.append((t, p.x, p.y, p.z, v.x, v.y, v.z))

    def _gt(self, msg: PointStamped):
        t = self._stamp(msg.header.stamp)
        self.gt.append((t, msg.point.x, msg.point.y, msg.point.z))

    def _twist(self, msg):
        t = time.time()
        self.vel.append((t, msg.linear.x, msg.linear.y, msg.linear.z))

    def _twist_st(self, msg: TwistStamped):
        t = self._stamp(msg.header.stamp)
        v = msg.twist.linear
        self.vel.append((t, v.x, v.y, v.z))


def _dedupe(rows, t_idx=0):
    if not rows:
        return rows
    import numpy as np

    a = np.asarray(rows, dtype=float)
    keys = (a[:, t_idx] * 1000.0).astype(np.int64)
    _, idx = np.unique(keys, return_index=True)
    return a[np.sort(idx)].tolist()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--stop-file', default='/tmp/a3d0_speed_stop')
    ap.add_argument('--max-sec', type=float, default=900.0)
    args = ap.parse_args()
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    stop = Path(args.stop_file)
    if stop.exists():
        stop.unlink()

    rclpy.init()
    node = Sampler()
    t0 = time.time()
    try:
        while rclpy.ok() and (time.time() - t0) < args.max_sec:
            rclpy.spin_once(node, timeout_sec=0.2)
            if stop.exists():
                break
    finally:
        vel = _dedupe(node.vel)
        odom = _dedupe(node.odom)
        gt = _dedupe(node.gt)
        with (out / 'body_velocity.csv').open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'vx', 'vy', 'vz'])
            w.writerows(vel)
        with (out / 'odom.csv').open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'z', 'vx', 'vy', 'vz'])
            w.writerows(odom)
        with (out / 'gt.csv').open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'z'])
            w.writerows(gt)
        print(f'vel={len(vel)} odom={len(odom)} gt={len(gt)}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
