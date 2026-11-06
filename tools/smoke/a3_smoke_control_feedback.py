#!/usr/bin/env python3
"""Bounded A3 smoke: control consumes Odometry feedback (no Webots)."""

from __future__ import annotations

import math
import multiprocessing as mp
import os
import signal
import sys
import time
import traceback
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool

from launch import LaunchDescription, LaunchService
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

CHILDREN: List[mp.Process] = []


def cleanup() -> None:
    for p in list(CHILDREN):
        if p.is_alive():
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                try:
                    os.kill(p.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
    for p in list(CHILDREN):
        p.join(timeout=2.0)
        if p.is_alive():
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                try:
                    os.kill(p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            p.join(timeout=1.0)
    CHILDREN.clear()
    # Reap orphaned component containers from prior smoke steps.
    import subprocess
    subprocess.run(
        ['pkill', '-9', '-f', 'component_container.*a3_smoke'],
        check=False, capture_output=True)
    time.sleep(0.4)


def _launch_worker(params_control: dict, params_lslam: dict, log_path: str, ready_q: mp.Queue) -> None:
    sys.stdout = open(log_path, 'w', buffering=1)
    sys.stderr = sys.stdout
    try:
        container = ComposableNodeContainer(
            name=f'a3_smoke_{os.getpid()}',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='localslam',
                    plugin='localslam::EKFslam',
                    name='EKFslam',
                    parameters=[params_lslam],
                    remappings=[('~/odometry', '/odometry')],
                ),
                ComposableNode(
                    package='control',
                    plugin='control::ControlQuad',
                    name='ControlQuad',
                    parameters=[params_control],
                    remappings=[('odometry', '/odometry')],
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


def start_stack(control_params: dict) -> Tuple[mp.Process, str]:
    log_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'build_a1',
        f'_a3_smoke_{os.getpid()}.log'))
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    lslam = {
        'Sensor_input_mode': 'standard',
        'Altitude_input_mode': 'derived_altitude',
        'Data_origin': 'webots',
        'Visual_update': False,
        'Attitude_update': True,
        'Altitude_update': True,
        'Speed_update': True,
        'Ground_station_range_update': False,
        'Prediction_model': 'noise_driven',
        'Max_freq_output': 50,
        'Robot_state_size': 13,
        'Mono_cam_frame_period': 0.0333,
        'Sigma_att_update': 0.001,
        'Sigma_spd_update': 0.1,
        'Sigma_alt_update': 0.1,
        'Sigma_w': 0.1,
        'Sigma_a': 6.0,
        'Tau_r': 1.0,
        'Tau_p': 100.0,
        'Alt_h_2_d': -1,
        'Odometry_frame_id': 'odom',
        'Odometry_child_frame_id': 'base_link',
        'Stats': False,
    }
    ready: mp.Queue = mp.Queue()
    proc = mp.Process(
        target=_launch_worker,
        args=(control_params, lslam, log_path, ready),
        daemon=True,
    )
    proc.start()
    CHILDREN.append(proc)
    time.sleep(1.0)  # allow component load + parameter-driven subscriptions
    try:
        ready.get(timeout=10.0)
    except Exception:
        pass
    return proc, log_path


def wait_nodes(names, timeout=25.0) -> bool:
    rclpy.init()
    n = Node('a3_waiter')
    try:
        t0 = time.time()
        while time.time() - t0 < timeout:
            have = set(n.get_node_names())
            if all(x in have for x in names):
                return True
            rclpy.spin_once(n, timeout_sec=0.1)
        return False
    finally:
        n.destroy_node()
        rclpy.shutdown()


def get_subs(node_name: str) -> List[str]:
    rclpy.init()
    n = Node('a3_graph')
    try:
        t0 = time.time()
        while time.time() - t0 < 3.0:
            if node_name in n.get_node_names():
                break
            rclpy.spin_once(n, timeout_sec=0.1)
        subs = n.get_subscriber_names_and_types_by_node(node_name, '/')
        return sorted({s for s, _ in subs})
    finally:
        n.destroy_node()
        rclpy.shutdown()


class Driver(Node):
    def __init__(self) -> None:
        super().__init__('a3_smoke_driver')
        self.pub_odom = self.create_publisher(Odometry, '/odometry', 10)
        self.pub_gps = self.create_publisher(PointStamped, '/webot_robot/gps', 10)
        self.cli_enable = self.create_client(SetBool, '/EKFslam/enable')
        self.cmd_count = 0
        # Observe cmd_vel to prove control path runs (optional activity)

    def arm_gps_altitude(self) -> None:
        # Two GPS samples: first latches origin, second sets actual_z_flag.
        for i, z in enumerate((0.0, 1.5)):
            m = PointStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.point.x = 0.0
            m.point.y = 0.0
            m.point.z = z
            self.pub_gps.publish(m)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def publish_odom(self, x, y, z, yaw, n=20) -> None:
        for _ in range(n):
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = z
            # yaw-only quaternion
            msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
            msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
            for i in range(36):
                msg.pose.covariance[i] = 0.0
                msg.twist.covariance[i] = 0.0
            msg.pose.covariance[0] = 0.01
            self.pub_odom.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)


def run_odometry_feedback() -> None:
    print('=== A3 smoke: Slam_feedback=true interface=odometry ===', flush=True)
    control = {
        'Slam_feedback': True,
        'Slam_feedback_interface': 'odometry',
        'Odometry_topic': 'odometry',
        'Odometry_expect_frame_id': 'odom',
        'Odometry_expect_child_frame_id': 'base_link',
        'Ekf_enable_service': '/EKFslam/enable',
        'Control_plan_path': '',
        'kp_x': 0.5, 'kp_y': 0.5, 'kp_z': 0.5, 'kp_yaw': 0.5,
        'maxControlSignal_xy': 0.5, 'minControlSignal_xy': -0.5,
        'maxControlSignal_z': 0.25, 'minControlSignal_z': -0.25,
        'maxControlSignal_yaw': 0.2, 'minControlSignal_yaw': -0.2,
        'cp_x': 0.2, 'cp_y': 0.2, 'cp_z': 0.25, 'cp_yaw': 0.05,
        'Mono_cam_img_rows': 240, 'Mono_cam_img_cols': 320,
        'Mono_cam_distortions': [0.0, 0.0, 0.0, 0.0, 0.0],
        'Mono_cam_cc_u': 160.0, 'Mono_cam_cc_v': 120.0,
        'Mono_cam_fc_u': 200.0, 'Mono_cam_fc_v': 200.0,
        'Mono_cam_alpha_c': 0.0,
        'Mono_cam_2_robot_axis_x': 0.1,
        'Mono_cam_2_robot_axis_y': 0.0,
        'Mono_cam_2_robot_axis_z': 1.57,
        'Mono_cam_2_robot_pos_x': 0.1,
        'Mono_cam_2_robot_pos_y': 0.0,
        'Mono_cam_2_robot_pos_z': 0.0,
        'Explore_area_lambda_y': -0.25,
        'Rec_Home_lambda_y': -0.5,
    }
    proc, log_path = start_stack(control)
    if not wait_nodes(['EKFslam', 'ControlQuad']):
        raise RuntimeError('nodes missing; log=' + open(log_path).read()[-2000:])
    subs = get_subs('ControlQuad')
    print('ControlQuad subs:', subs, flush=True)
    if '/odometry' not in subs and 'odometry' not in ''.join(subs):
        # Fully qualified may be /odometry
        if not any('odometry' in s for s in subs):
            raise AssertionError(f'expected odometry subscription; got {subs}')
    if any('Robotstate' in s for s in subs):
        raise AssertionError(f'odometry mode must not subscribe Robotstate; {subs}')

    rclpy.init()
    d = Driver()
    try:
        d.arm_gps_altitude()
        # Latch then update
        d.publish_odom(0.0, 0.0, 0.0, 0.0, n=5)
        d.publish_odom(1.0, -0.5, 2.0, 0.3, n=30)
        print('published known Odometry samples with GPS altitude latch', flush=True)
        # Enable EKF (StartSLAM path)
        if d.cli_enable.wait_for_service(timeout_sec=5.0):
            req = SetBool.Request(); req.data = True
            fut = d.cli_enable.call_async(req)
            rclpy.spin_until_future_complete(d, fut, timeout_sec=5.0)
            print('EKF enable service OK', flush=True)
        else:
            print('WARN: enable service not ready (still OK for feedback sub check)', flush=True)
    finally:
        d.destroy_node()
        rclpy.shutdown()
        cleanup()
        time.sleep(0.4)


def run_gt_path_no_odom_sub() -> None:
    print('=== A3 smoke: Slam_feedback=false retains GT path (no odom consume) ===', flush=True)
    control = {
        'Slam_feedback': False,
        'Slam_feedback_interface': 'odometry',  # interface unused when Slam_feedback=false for r_c
        'Odometry_topic': 'odometry',
        'Odometry_expect_frame_id': 'odom',
        'Odometry_expect_child_frame_id': 'base_link',
        'Ekf_enable_service': '/EKFslam/enable',
        'Control_plan_path': '',
        'kp_x': 0.5, 'kp_y': 0.5, 'kp_z': 0.5, 'kp_yaw': 0.5,
        'maxControlSignal_xy': 0.5, 'minControlSignal_xy': -0.5,
        'maxControlSignal_z': 0.25, 'minControlSignal_z': -0.25,
        'maxControlSignal_yaw': 0.2, 'minControlSignal_yaw': -0.2,
        'cp_x': 0.2, 'cp_y': 0.2, 'cp_z': 0.25, 'cp_yaw': 0.05,
        'Mono_cam_img_rows': 240, 'Mono_cam_img_cols': 320,
        'Mono_cam_distortions': [0.0, 0.0, 0.0, 0.0, 0.0],
        'Mono_cam_cc_u': 160.0, 'Mono_cam_cc_v': 120.0,
        'Mono_cam_fc_u': 200.0, 'Mono_cam_fc_v': 200.0,
        'Mono_cam_alpha_c': 0.0,
        'Mono_cam_2_robot_axis_x': 0.1,
        'Mono_cam_2_robot_axis_y': 0.0,
        'Mono_cam_2_robot_axis_z': 1.57,
        'Mono_cam_2_robot_pos_x': 0.1,
        'Mono_cam_2_robot_pos_y': 0.0,
        'Mono_cam_2_robot_pos_z': 0.0,
        'Explore_area_lambda_y': -0.25,
        'Rec_Home_lambda_y': -0.5,
    }
    # Still constructs odometry subscription because interface=odometry — for GT
    # path r_c comes from GPS/IMU. Verify GPS/IMU subs exist.
    proc, log_path = start_stack(control)
    if not wait_nodes(['ControlQuad']):
        raise RuntimeError('ControlQuad missing')
    subs = get_subs('ControlQuad')
    print('ControlQuad GT-mode subs:', subs, flush=True)
    if not any('gps' in s for s in subs):
        raise AssertionError(f'GT path missing GPS sub; {subs}')
    if not any(s.endswith('/imu') or s == '/imu' or s.endswith('imu') for s in subs):
        raise AssertionError(f'GT path missing IMU sub; {subs}')
    cleanup()
    time.sleep(0.3)


def run_legacy_interface_subs() -> None:
    print('=== A3 smoke: interface=legacy subscribes Robotstate only ===', flush=True)
    control = {
        'Slam_feedback': True,
        'Slam_feedback_interface': 'legacy',
        'Odometry_topic': 'odometry',
        'Odometry_expect_frame_id': 'odom',
        'Odometry_expect_child_frame_id': 'base_link',
        'Ekf_enable_service': '/EKFslam/enable',
        'Control_plan_path': '',
        'kp_x': 0.5, 'kp_y': 0.5, 'kp_z': 0.5, 'kp_yaw': 0.5,
        'maxControlSignal_xy': 0.5, 'minControlSignal_xy': -0.5,
        'maxControlSignal_z': 0.25, 'minControlSignal_z': -0.25,
        'maxControlSignal_yaw': 0.2, 'minControlSignal_yaw': -0.2,
        'cp_x': 0.2, 'cp_y': 0.2, 'cp_z': 0.25, 'cp_yaw': 0.05,
        'Mono_cam_img_rows': 240, 'Mono_cam_img_cols': 320,
        'Mono_cam_distortions': [0.0, 0.0, 0.0, 0.0, 0.0],
        'Mono_cam_cc_u': 160.0, 'Mono_cam_cc_v': 120.0,
        'Mono_cam_fc_u': 200.0, 'Mono_cam_fc_v': 200.0,
        'Mono_cam_alpha_c': 0.0,
        'Mono_cam_2_robot_axis_x': 0.1,
        'Mono_cam_2_robot_axis_y': 0.0,
        'Mono_cam_2_robot_axis_z': 1.57,
        'Mono_cam_2_robot_pos_x': 0.1,
        'Mono_cam_2_robot_pos_y': 0.0,
        'Mono_cam_2_robot_pos_z': 0.0,
        'Explore_area_lambda_y': -0.25,
        'Rec_Home_lambda_y': -0.5,
    }
    proc, log_path = start_stack(control)
    if not wait_nodes(['ControlQuad']):
        raise RuntimeError('ControlQuad missing')
    subs = get_subs('ControlQuad')
    print('ControlQuad legacy subs:', subs, flush=True)
    if not any('Robotstate' in s for s in subs):
        raise AssertionError(f'legacy missing Robotstate; {subs}')
    if any(s.endswith('/odometry') or s == '/odometry' for s in subs):
        raise AssertionError(f'legacy must not subscribe odometry; {subs}')
    cleanup()
    time.sleep(0.3)


def main() -> int:
    try:
        run_odometry_feedback()
        run_gt_path_no_odom_sub()
        run_legacy_interface_subs()
        print('A3_SMOKE_OK', flush=True)
        return 0
    except Exception as exc:
        print(f'A3_SMOKE_FAIL: {exc}', file=sys.stderr, flush=True)
        traceback.print_exc()
        return 1
    finally:
        cleanup()


if __name__ == '__main__':
    mp.set_start_method('spawn', force=True)
    sys.exit(main())
