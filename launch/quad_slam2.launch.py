# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Prefer launch_a1.sh + ekf_monoslam_bringup for A1 (install_a1 + share paths).
# This file remains for repo-root workflows; it resolves bringup share when
# available and otherwise falls back to cwd config/.

import os

import launch
import yaml
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _load_config():
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('ekf_monoslam_bringup')
        config_file = os.path.join(share, 'config', 'quad_params.yaml')
        default_plan = os.path.join(share, 'control_plan.txt')
        return config_file, default_plan
    except Exception:
        config_file = os.path.join(os.getcwd(), 'config', 'quad_params.yaml')
        default_plan = os.path.join(os.getcwd(), 'control_plan.txt')
        return config_file, default_plan


def generate_launch_description():
    config_file, default_plan = _load_config()

    with open(config_file, 'r') as file:
        raw = yaml.safe_load(file)

    lslam_params = dict(raw['/lslam']['ros__parameters'])
    plot_params = dict(raw['/plot2']['ros__parameters'])
    webotbridge_params = dict(raw['/webotbridge']['ros__parameters'])
    control_params = dict(raw['/control']['ros__parameters'])

    plan_param = control_params.get('Control_plan_path', '')
    if (not plan_param) or (not os.path.isabs(str(plan_param))) or (
            not os.path.isfile(plan_param)):
        control_params['Control_plan_path'] = default_plan

    lslam_remaps = [
        ('~/image', '/image'),
        ('~/imu', '/imu_out'),
        ('~/body_velocity', '/body_velocity'),
        ('~/downward_range', '/downward_range'),
        ('~/altitude', '/altitude'),
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
                    parameters=[webotbridge_params]),
                ComposableNode(
                    package='control',
                    plugin='control::ControlQuad',
                    name='ControlQuad',
                    parameters=[control_params]),
            ],
            output='screen',
    )
    return launch.LaunchDescription([container])
