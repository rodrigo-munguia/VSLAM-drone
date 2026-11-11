#!/usr/bin/env python3
"""G2: real globalslam::Gslam lifecycle smoke (corrections remain off).

Exercises load/spin, enable/disable/re-enable, standard + legacy reset,
repeated reset, unload/reload, shutdown with both workers, and operation
with Local SLAM / control services absent (fake counters verify zero outbound
LSposUpdate and Control 'c' while Apply_global_corrections=false).

Optionally publishes a minimal Kf fixture while enabled to exercise worker
ingest (idle-worker-only if publishing is skipped). Does not enable
corrections. Does not start Webots / Local SLAM / launch2.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import tempfile
import time
import traceback
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from geometry_msgs.msg import Transform
from std_srvs.srv import SetBool, Trigger
from interfaces.msg import Kf
from interfaces.srv import LSposUpdate, SimpleServ

IMG_H, IMG_W = 240, 320
TIMEOUT_S = 90.0


def make_gray_frame() -> bytes:
    # Minimal textured-ish frame (not a full BA fixture).
    buf = bytearray(IMG_H * IMG_W)
    for v in range(IMG_H):
        for u in range(IMG_W):
            buf[v * IMG_W + u] = ((u // 16) + (v // 16)) % 2 * 200 + 20
    return bytes(buf)


def make_kf_msg(node: Node, frame: bytes) -> Kf:
    msg = Kf()
    msg.img.header.stamp = node.get_clock().now().to_msg()
    msg.img.height = IMG_H
    msg.img.width = IMG_W
    msg.img.encoding = 'mono8'
    msg.img.is_bigendian = 0
    msg.img.step = IMG_W
    msg.img.data = frame
    identity = Transform()
    identity.rotation.w = 1.0
    msg.r2c = identity
    msg.n2r = identity
    msg.radius_uncertainty = 0.0
    return msg


class FakeDownstream(Node):
    """Counts LSposUpdate / Control 'c' so we can assert zero outbound."""

    def __init__(self) -> None:
        super().__init__('g2_fake_downstream')
        self.ls_n = 0
        self.ctrl_c_n = 0
        self.create_service(
            LSposUpdate, 'local_slam_pos_update', self._on_ls)
        self.create_service(
            SimpleServ, 'control_run_service', self._on_ctrl)

    def _on_ls(self, request, response):
        self.ls_n += 1
        self.get_logger().warn(
            f'UNEXPECTED LSposUpdate type={request.type} '
            f'delta=({request.delta_x},{request.delta_y},{request.delta_z})')
        response.accepted = False
        response.applied = False
        response.outcome = 'rejected'
        response.reason = 'g2_smoke_sink'
        return response

    def _on_ctrl(self, request, response):
        if request.cmd == 'c':
            self.ctrl_c_n += 1
            self.get_logger().warn("UNEXPECTED Control 'c'")
        response.response = True
        return response


class Driver(Node):
    def __init__(self) -> None:
        super().__init__('g2_globalslam_driver')
        self.pub_kf = self.create_publisher(Kf, 'Kf_topic', 10)
        self.pub_kf_cl = self.create_publisher(Kf, 'Kf_cl_topic', 10)
        self.cli_enable = self.create_client(SetBool, '/gslam/enable')
        self.cli_reset = self.create_client(Trigger, '/gslam/reset')
        self.cli_legacy = self.create_client(SimpleServ, 'globalslam_run_service')

    def wait_services(self, timeout: float) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if (self.cli_enable.wait_for_service(timeout_sec=0.5) and
                    self.cli_reset.wait_for_service(timeout_sec=0.2) and
                    self.cli_legacy.wait_for_service(timeout_sec=0.2)):
                return True
        return False

    def call_enable(self, on: bool, timeout: float = 5.0) -> bool:
        req = SetBool.Request()
        req.data = on
        fut = self.cli_enable.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.done() and bool(fut.result() and fut.result().success)

    def call_reset(self, timeout: float = 5.0) -> bool:
        fut = self.cli_reset.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.done() and bool(fut.result() and fut.result().success)

    def call_legacy_i(self, timeout: float = 5.0) -> bool:
        req = SimpleServ.Request()
        req.cmd = ord('i')  # interfaces/SimpleServ uses int64 cmd
        fut = self.cli_legacy.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.done() and bool(fut.result() and fut.result().response)

    def publish_minimal_kfs(self, n: int = 3) -> None:
        frame = make_gray_frame()
        for _ in range(n):
            msg = make_kf_msg(self, frame)
            self.pub_kf.publish(msg)
            self.pub_kf_cl.publish(msg)
            time.sleep(0.05)


def launch_globalslam(env: dict) -> subprocess.Popen:
    return subprocess.Popen(
        ['ros2', 'launch', 'globalslam', 'globalslam.launch.py'],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=env,
        preexec_fn=os.setsid,
        cwd='/tmp',
    )


def kill_proc(proc: Optional[subprocess.Popen]) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=3)


def _first_existing(*candidates: str) -> Optional[str]:
    for path in candidates:
        if os.path.isfile(path):
            return path
    return None


def check_component_artifacts(install_prefix: str) -> None:
    # Support merged install/ and colcon isolated install/<pkg>/.
    lib = _first_existing(
        os.path.join(install_prefix, 'lib', 'libglobalslam_component.so'),
        os.path.join(install_prefix, 'globalslam', 'lib', 'libglobalslam_component.so'),
    )
    if lib is None:
        raise RuntimeError(
            f'missing component library under {install_prefix} '
            '(expected lib/libglobalslam_component.so)')
    idx = _first_existing(
        os.path.join(
            install_prefix, 'share', 'ament_index', 'resource_index',
            'rclcpp_components', 'globalslam'),
        os.path.join(
            install_prefix, 'globalslam', 'share', 'ament_index',
            'resource_index', 'rclcpp_components', 'globalslam'),
    )
    if idx is None:
        raise RuntimeError(f'missing component registration under {install_prefix}')
    with open(idx, 'r', encoding='utf-8') as f:
        body = f.read()
    if 'globalslam::Gslam' not in body:
        raise RuntimeError(f'registration missing globalslam::Gslam: {body!r}')
    # Runtime linker resolution (GTSAM via LD_LIBRARY_PATH)
    out = subprocess.check_output(['ldd', lib], text=True)
    if 'not found' in out:
        raise RuntimeError(f'unresolved deps for {lib}:\n{out}')
    if 'libgtsam' not in out:
        raise RuntimeError(f'libgtsam not linked in {lib}:\n{out}')


def run_lifecycle(env: dict) -> dict:
    results = {
        'workers_exercised': 'minimal_input',  # or idle if publish fails
        'ls_n': 0,
        'ctrl_c_n': 0,
    }
    procs: List[subprocess.Popen] = []
    rclpy.init()
    fake = FakeDownstream()
    driver = Driver()
    execu = SingleThreadedExecutor()
    execu.add_node(fake)
    execu.add_node(driver)

    def spin_for(seconds: float) -> None:
        t0 = time.time()
        while time.time() - t0 < seconds:
            execu.spin_once(timeout_sec=0.1)

    try:
        # --- load #1 ---
        p = launch_globalslam(env)
        procs.append(p)
        spin_for(1.0)
        if not driver.wait_services(25.0):
            raise RuntimeError('gslam services not available after launch')

        # enable / disable / re-enable
        if not driver.call_enable(True):
            raise RuntimeError('enable true failed')
        spin_for(0.5)
        if not driver.call_enable(False):
            raise RuntimeError('enable false failed')
        spin_for(0.3)
        if not driver.call_enable(True):
            raise RuntimeError('re-enable failed')

        # minimal input fixture (worker ingest; BA may no-op on thin data)
        driver.publish_minimal_kfs(4)
        spin_for(1.5)

        # standard reset + repeated
        if not driver.call_reset():
            raise RuntimeError('~/reset failed')
        if not driver.call_reset():
            raise RuntimeError('repeated ~/reset failed')

        # legacy reset
        if not driver.call_legacy_i():
            raise RuntimeError("legacy SimpleServ 'i' reset failed")

        # operate with downstream present but corrections gated off
        driver.publish_minimal_kfs(2)
        spin_for(1.0)

        # shutdown with both workers
        kill_proc(p)
        procs.pop()
        spin_for(0.5)

        # --- unload/reload ---
        p2 = launch_globalslam(env)
        procs.append(p2)
        if not driver.wait_services(25.0):
            raise RuntimeError('gslam services not available after reload')
        if not driver.call_enable(True):
            raise RuntimeError('enable after reload failed')
        driver.publish_minimal_kfs(2)
        spin_for(1.0)
        if not driver.call_reset():
            raise RuntimeError('reset after reload failed')
        kill_proc(p2)
        procs.pop()

        results['ls_n'] = fake.ls_n
        results['ctrl_c_n'] = fake.ctrl_c_n
        if fake.ls_n != 0 or fake.ctrl_c_n != 0:
            raise RuntimeError(
                f'correction gate leaked: ls={fake.ls_n} ctrl_c={fake.ctrl_c_n}')

        # --- absent Local SLAM / control services (no hang on shutdown) ---
        execu.remove_node(fake)
        fake.destroy_node()
        fake = None
        spin_for(0.5)
        p3 = launch_globalslam(env)
        procs.append(p3)
        if not driver.wait_services(25.0):
            raise RuntimeError('gslam services not available without downstream')
        # Best-effort enable/reset; focus is no hang when downstream is absent.
        _ = driver.call_enable(True)
        driver.publish_minimal_kfs(2)
        spin_for(1.0)
        _ = driver.call_reset()
        t_shutdown = time.time()
        kill_proc(p3)
        procs.pop()
        if time.time() - t_shutdown > 10.0:
            raise RuntimeError('shutdown waited too long without downstream services')

        return results
    finally:
        for p in procs:
            kill_proc(p)
        try:
            execu.remove_node(driver)
        except Exception:
            pass
        if fake is not None:
            try:
                execu.remove_node(fake)
            except Exception:
                pass
            fake.destroy_node()
        driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    t0 = time.time()
    install = os.environ.get(
        'EKF_INSTALL_BASE',
        os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            'install'))
    if not os.path.isdir(install):
        print(f'FAIL: install prefix missing: {install}', file=sys.stderr)
        return 2

    # Ensure overlay + GTSAM runtime path are present in child env.
    env = os.environ.copy()
    setup = os.path.join(install, 'setup.bash')
    if not os.path.isfile(setup):
        print(f'FAIL: {setup} missing — source install/setup.bash first', file=sys.stderr)
        return 2

    try:
        check_component_artifacts(install)
        print('OK: component library, registration, and ldd deps')
        results = run_lifecycle(env)
        elapsed = time.time() - t0
        print(
            f'PASS G2 smoke: workers={results["workers_exercised"]} '
            f'ls_outbound={results["ls_n"]} ctrl_c_outbound={results["ctrl_c_n"]} '
            f'elapsed={elapsed:.1f}s')
        return 0
    except Exception as exc:
        print(f'FAIL G2 smoke: {exc}', file=sys.stderr)
        traceback.print_exc()
        return 1
    finally:
        # Hard bound: do not hang the session.
        if time.time() - t0 > TIMEOUT_S:
            print('FAIL: overall timeout', file=sys.stderr)


if __name__ == '__main__':
    sys.exit(main())
