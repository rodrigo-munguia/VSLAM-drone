#!/usr/bin/env python3
"""Standalone Global SLAM (G2/G4).

Resolves installed share resources (cwd-independent). Corrections default off.
Does not start Webots, Local SLAM, control, or viewers.

  source install/setup.bash   # plus GTSAM LD_LIBRARY_PATH
  ros2 launch globalslam globalslam.launch.py

Services:
  ros2 service call /gslam/enable std_srvs/srv/SetBool "{data: true}"
  ros2 service call /gslam/reset std_srvs/srv/Trigger "{}"

Public viz (G4): /gslam/global_keyframes (Path), /gslam/global_landmarks (PointCloud2).
Remap via global_keyframes_topic / global_landmarks_topic.

Meaningful map processing requires Mono_cam_* consistent with Local SLAM YAML
(full intrinsics, not resolution alone). CameraInfo is not wired.
"""

import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def _build(context, *args, **kwargs):
    share = get_package_share_directory('globalslam')
    default_params = os.path.join(share, 'config', 'globalslam_params.yaml')
    params_file = LaunchConfiguration('params_file').perform(context) or default_params
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in (
        '1', 'true', 'yes')

    with open(params_file, 'r') as f:
        raw = yaml.safe_load(f)
    gslam_params = dict(raw.get('/gslam', {}).get('ros__parameters', {}))
    gslam_params['use_sim_time'] = use_sim_time
    # Hard safety: standalone never opens the correction gate unless YAML says so.
    gslam_params.setdefault('Apply_global_corrections', False)
    gslam_params.setdefault('Processing_start_enabled', False)

    remaps = []
    kf = LaunchConfiguration('kf_topic').perform(context)
    kf_cl = LaunchConfiguration('kf_cl_topic').perform(context)
    gkf = LaunchConfiguration('global_keyframes_topic').perform(context)
    glm = LaunchConfiguration('global_landmarks_topic').perform(context)
    if kf:
        remaps.append(('Kf_topic', kf))
    if kf_cl:
        remaps.append(('Kf_cl_topic', kf_cl))
    if gkf:
        remaps.append(('~/global_keyframes', gkf))
    if glm:
        remaps.append(('~/global_landmarks', glm))

    container = ComposableNodeContainer(
        name='globalslam_standalone_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='globalslam',
                plugin='globalslam::Gslam',
                name='gslam',
                parameters=[gslam_params],
                remappings=remaps),
        ],
        output='screen',
    )
    return [container]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Absolute path to YAML. Empty = installed globalslam_params.yaml.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'kf_topic',
            default_value='',
            description='Remap Kf_topic. Empty = default Kf_topic.'),
        DeclareLaunchArgument(
            'kf_cl_topic',
            default_value='',
            description='Remap Kf_cl_topic. Empty = default Kf_cl_topic.'),
        DeclareLaunchArgument(
            'global_keyframes_topic',
            default_value='',
            description='Remap ~/global_keyframes. Empty = /gslam/global_keyframes.'),
        DeclareLaunchArgument(
            'global_landmarks_topic',
            default_value='',
            description='Remap ~/global_landmarks. Empty = /gslam/global_landmarks.'),
        OpaqueFunction(function=_build),
    ])
