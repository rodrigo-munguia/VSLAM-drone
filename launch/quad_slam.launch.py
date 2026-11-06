# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Legacy multi-component launch (includes Global SLAM). Prefer ./launch2.sh
# for the active Webots + Local SLAM + Plot2/RViz experiment.
# The obsolete OpenCV `plot` package has been removed; this file uses plot2.

import os
import launch
import yaml
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    config_file = os.path.join(os.getcwd(), 'config', 'quad_params.yaml')

    with open(config_file, 'r') as file:
        raw = yaml.safe_load(file)

    lslam_params = dict(raw['/lslam']['ros__parameters'])
    plot_params = dict(raw.get('/plot2', {}).get('ros__parameters', {}))
    gslam_params = dict(raw['/gslam']['ros__parameters'])
    webotbridge_params = dict(raw['/webotbridge']['ros__parameters'])
    control_params = dict(raw['/control']['ros__parameters'])

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
                parameters=[lslam_params]),
            ComposableNode(
                package='globalslam',
                plugin='globalslam::Gslam',
                name='Gslam',
                parameters=[gslam_params]),
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
