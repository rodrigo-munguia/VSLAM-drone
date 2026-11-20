#!/usr/bin/env python3
"""Sample /GT_topic until stop file appears."""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class Sampler(Node):
    def __init__(self):
        super().__init__('a3d0_gt_sampler')
        self.rows = []
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.create_subscription(PointStamped, '/GT_topic', self._cb, qos)

    def _cb(self, msg: PointStamped):
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.rows.append((t, msg.point.x, msg.point.y, msg.point.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--stop-file', default='/tmp/a3d0_smoke_stop')
    ap.add_argument('--max-sec', type=float, default=600.0)
    args = ap.parse_args()
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
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open('w') as f:
            f.write('t,x,y,z\n')
            for r in node.rows:
                f.write(f'{r[0]},{r[1]},{r[2]},{r[3]}\n')
        node.destroy_node()
        rclpy.shutdown()
        print(f'wrote {len(node.rows)} samples -> {out}')


if __name__ == '__main__':
    main()
