#!/usr/bin/env python3
"""Integrated quad experiment launch (no Global SLAM).

Webots is launched separately (see ./launch2.sh).

Viewer selection (propagated from launch2.sh):
  use_plot2:=true|false   Plot2 inside the app container (default true)

RViz2 is started by launch2.sh as a separate process when --viewer rviz2.
"""

import os

import launch
import yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _build(context, *args, **kwargs):
    bringup_share = get_package_share_directory('ekf_monoslam_bringup')
    config_file = os.path.join(bringup_share, 'config', 'quad_params.yaml')
    default_plan = os.path.join(bringup_share, 'control_plan.txt')
    use_plot2 = LaunchConfiguration('use_plot2').perform(context).lower() in (
        '1', 'true', 'yes')
    slam_feedback = LaunchConfiguration('slam_feedback').perform(context)
    slam_iface = LaunchConfiguration('slam_feedback_interface').perform(context)

    with open(config_file, 'r') as file:
        raw = yaml.safe_load(file)

    lslam_params = dict(raw['/lslam']['ros__parameters'])
    webotbridge_params = dict(raw['/webotbridge']['ros__parameters'])
    control_params = dict(raw['/control']['ros__parameters'])

    if slam_feedback != '':
        control_params['Slam_feedback'] = slam_feedback.lower() in ('1', 'true', 'yes')
    if slam_iface != '':
        control_params['Slam_feedback_interface'] = slam_iface

    plan_override = LaunchConfiguration('control_plan').perform(context)
    if plan_override:
        control_params['Control_plan_path'] = plan_override
    plan_param = control_params.get('Control_plan_path', '')
    if (not plan_param) or (not os.path.isabs(str(plan_param))) or (
            not os.path.isfile(str(plan_param))):
        control_params['Control_plan_path'] = default_plan

    for p in (lslam_params, webotbridge_params, control_params):
        p['use_sim_time'] = True

    calibration_source = LaunchConfiguration('calibration_source').perform(context)
    if calibration_source:
        lslam_params['Calibration_source'] = calibration_source

    lslam_remaps = [
        ('~/image', '/image'),
        ('~/camera_info', '/camera_info'),
        ('~/imu', '/imu_out'),
        ('~/body_velocity', '/body_velocity'),
        ('~/downward_range', '/downward_range'),
        ('~/altitude', '/altitude'),
        ('~/odometry', '/odometry'),
        ('~/landmarks', '/EKFslam/landmarks'),
        ('~/path', '/EKFslam/path'),
        ('~/estimator_reference', '/EKFslam/estimator_reference'),
    ]
    webotbridge_remaps = [
        ('camera', '/camera/image_color'),
        ('camera_info', '/camera_info'),
        ('imu', '/imu'),
        ('ds0', '/ds0'),
        ('EKFslam/estimator_reference', '/EKFslam/estimator_reference'),
        ('ground_truth/path', '/ground_truth/path'),
    ]
    control_remaps = [
        ('odometry', '/odometry'),
    ]
    plot_remaps = [
        ('odometry', '/odometry'),
        ('landmarks', '/EKFslam/landmarks'),
        ('path', '/EKFslam/path'),
    ]

    nodes = [
        ComposableNode(
            package='localslam',
            plugin='localslam::EKFslam',
            name='EKFslam',
            parameters=[lslam_params],
            remappings=lslam_remaps),
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
    ]

    if use_plot2:
        try:
            get_package_share_directory('plot2')
            plot_params = dict(raw.get('/plot2', {}).get('ros__parameters', {}))
            plot_params['use_sim_time'] = True
            nodes.insert(1, ComposableNode(
                package='plot2',
                plugin='plot2::PLOTscene2',
                name='PLOTscene2',
                parameters=[plot_params],
                remappings=plot_remaps))
        except PackageNotFoundError:
            raise RuntimeError(
                'use_plot2:=true but plot2 is not installed in this overlay')

    container = ComposableNodeContainer(
        name='quad_experiment_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=nodes,
        output='screen',
    )
    return [container]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('use_plot2', default_value='true'),
        DeclareLaunchArgument(
            'slam_feedback',
            default_value='',
            description='Override Slam_feedback (true/false). Empty = YAML default.'),
        DeclareLaunchArgument(
            'slam_feedback_interface',
            default_value='',
            description='Override interface (odometry|legacy). Empty = YAML default.'),
        DeclareLaunchArgument(
            'calibration_source',
            default_value='',
            description="Override Calibration_source (yaml|camera_info). Empty = YAML default."),
        DeclareLaunchArgument(
            'control_plan',
            default_value='',
            description='Absolute path to control plan. Empty = installed default / YAML.'),
        OpaqueFunction(function=_build),
    ])
