#!/usr/bin/env python3
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Launch Local SLAM A1 experiment using the A1 install tree resources.
# Resolves control_plan.txt and config from ekf_monoslam_bringup share
# (cwd-independent). Explicit Control_plan_path in YAML still overrides
# when absolute; this launch always injects the installed default path.

import os

import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    bringup_share = get_package_share_directory('ekf_monoslam_bringup')
    config_file = os.path.join(bringup_share, 'config', 'quad_params.yaml')
    default_plan = os.path.join(bringup_share, 'control_plan.txt')

    with open(config_file, 'r') as file:
        raw = yaml.safe_load(file)

    lslam_params = dict(raw['/lslam']['ros__parameters'])
    plot_params = dict(raw['/plot2']['ros__parameters'])
    webotbridge_params = dict(raw['/webotbridge']['ros__parameters'])
    control_params = dict(raw['/control']['ros__parameters'])

    # Installed resource wins over cwd-relative YAML default; callers may still
    # override via launch remapping/parameters if needed.
    plan_param = control_params.get('Control_plan_path', '')
    if (not plan_param) or (not os.path.isabs(plan_param)) or (not os.path.isfile(plan_param)):
        control_params['Control_plan_path'] = default_plan

    lslam_remaps = [
        ('~/image', '/image'),
        ('~/imu', '/imu_out'),
        ('~/body_velocity', '/body_velocity'),
        ('~/downward_range', '/downward_range'),
        ('~/altitude', '/altitude'),
        ('~/odometry', '/odometry'),
        ('~/estimator_reference', '/EKFslam/estimator_reference'),
    ]
    webotbridge_remaps = [
        ('EKFslam/estimator_reference', '/EKFslam/estimator_reference'),
        ('ground_truth/path', '/ground_truth/path'),
    ]
    control_remaps = [
        # A3: controller Odometry_topic default "odometry" → /odometry from Local SLAM
        ('odometry', '/odometry'),
    ]

    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='localslam',
                plugin='localslam::EKFslam',
                name='EKFslam',
                parameters=[lslam_params],
                remappings=lslam_remaps),
            ComposableNode(
                package='plot2',
                plugin='plot2::PLOTscene2',
                name='PLOTscene2',
                parameters=[plot_params]),
            ComposableNode(
                package='webotbridge',
                plugin='webotbridge::WEBOTb',
                name='WEBOTb',
                parameters=[webotbridge_params],
                remappings=webotbridge_remaps),
            ComposableNode(
                package='control',
                plugin='control::ControlQuad',
                name='ControlQuad',
                parameters=[control_params],
                remappings=control_remaps),
        ],
        output='screen',
    )
    return launch.LaunchDescription([container])
