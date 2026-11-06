#!/usr/bin/env python3
"""Standalone Local SLAM launch (no Webots, control, plot, keyboard, or Global SLAM).

Installed with package `localslam`. Does not depend on ekf_monoslam_bringup.

Defaults:
  Sensor_input_mode:=standard (migrated sensor_msgs / mavros Altitude inputs)
  use_sim_time:=false (set true when a /clock publisher is present)

Initialization / enable / reset (after launch):
  # EKF worker starts disabled; enable when sensors are ready (or before):
  ros2 service call /EKFslam/enable std_srvs/srv/SetBool "{data: true}"
  # Re-initialize estimator state:
  ros2 service call /EKFslam/reset std_srvs/srv/Trigger "{}"
  # Legacy SimpleServ still available: ekf_run_service cmd 'r'/'s'/'i'

The node waits for measurements on its subscriptions; it does not call
unavailable application services or load mission plans.
"""

import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def _bool(text: str) -> bool:
    return text.lower() in ('1', 'true', 'yes', 'on')


def _build(context, *args, **kwargs):
    share = get_package_share_directory('localslam')
    default_yaml = os.path.join(share, 'config', 'localslam_params.yaml')
    params_file = LaunchConfiguration('params_file').perform(context)
    if not params_file:
        params_file = default_yaml

    with open(params_file, 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f)

    # Accept either /** or /lslam or /EKFslam style roots from older YAML.
    if '/**' in raw and 'ros__parameters' in raw['/**']:
        params = dict(raw['/**']['ros__parameters'])
    elif '/lslam' in raw and 'ros__parameters' in raw['/lslam']:
        params = dict(raw['/lslam']['ros__parameters'])
    elif '/EKFslam' in raw and 'ros__parameters' in raw['/EKFslam']:
        params = dict(raw['/EKFslam']['ros__parameters'])
    else:
        # Flat dict of parameters
        params = dict(raw.get('ros__parameters', raw))

    # Launch overrides (empty string = leave YAML value).
    sensor_mode = LaunchConfiguration('sensor_input_mode').perform(context)
    if sensor_mode:
        params['Sensor_input_mode'] = sensor_mode
    alt_mode = LaunchConfiguration('altitude_input_mode').perform(context)
    if alt_mode:
        params['Altitude_input_mode'] = alt_mode
    for arg, key, cast in (
        ('pressure_t0_k', 'Pressure_t0_K', float),
        ('pressure_min_pa', 'Pressure_min_Pa', float),
        ('pressure_max_pa', 'Pressure_max_Pa', float),
        ('pressure_z_lever_arm_m', 'Pressure_z_lever_arm_m', float),
        ('pressure_waiting_bound', 'Pressure_waiting_bound', int),
    ):
        val = LaunchConfiguration(arg).perform(context)
        if val != '':
            params[key] = cast(val)
    pref = LaunchConfiguration('pressure_reference_policy').perform(context)
    if pref:
        params['Pressure_reference_policy'] = pref
    pframe = LaunchConfiguration('pressure_expected_frame_id').perform(context)
    if pframe != '':
        params['Pressure_expected_frame_id'] = pframe
    cal_source = LaunchConfiguration('calibration_source').perform(context)
    if cal_source:
        params['Calibration_source'] = cal_source
    cal_path = LaunchConfiguration('camera_info_image_path').perform(context)
    if cal_path:
        params['Camera_info_image_path'] = cal_path
    cal_frame = LaunchConfiguration('camera_info_expected_frame_id').perform(context)
    if cal_frame != '':
        params['Camera_info_expected_frame_id'] = cal_frame
    cal_qos = LaunchConfiguration('camera_info_qos_reliability').perform(context)
    if cal_qos:
        params['Camera_info_qos_reliability'] = cal_qos
    odom_frame = LaunchConfiguration('odometry_frame_id').perform(context)
    if odom_frame:
        params['Odometry_frame_id'] = odom_frame
    odom_child = LaunchConfiguration('odometry_child_frame_id').perform(context)
    if odom_child:
        params['Odometry_child_frame_id'] = odom_child
    use_sim = LaunchConfiguration('use_sim_time').perform(context)
    if use_sim != '':
        params['use_sim_time'] = _bool(use_sim)
    visual = LaunchConfiguration('visual_update').perform(context)
    if visual != '':
        params['Visual_update'] = _bool(visual)
    gs_range = LaunchConfiguration('ground_station_range_update').perform(context)
    if gs_range != '':
        params['Ground_station_range_update'] = _bool(gs_range)

    # Optional noise overrides (empty = YAML).
    for arg, key in (
        ('sigma_att_update', 'Sigma_att_update'),
        ('sigma_spd_update', 'Sigma_spd_update'),
        ('sigma_alt_update', 'Sigma_alt_update'),
        ('sigma_cam_uv', 'Sigma_cam_uv'),
        ('sigma_w', 'Sigma_w'),
        ('sigma_a', 'Sigma_a'),
    ):
        val = LaunchConfiguration(arg).perform(context)
        if val != '':
            params[key] = float(val)

    remappings = [
        ('~/image', LaunchConfiguration('image_topic').perform(context)),
        ('~/camera_info', LaunchConfiguration('camera_info_topic').perform(context)),
        ('~/imu', LaunchConfiguration('imu_topic').perform(context)),
        ('~/body_velocity', LaunchConfiguration('body_velocity_topic').perform(context)),
        ('~/downward_range', LaunchConfiguration('downward_range_topic').perform(context)),
        ('~/altitude', LaunchConfiguration('altitude_topic').perform(context)),
        ('~/pressure', LaunchConfiguration('pressure_topic').perform(context)),
        ('~/odometry', LaunchConfiguration('odometry_topic').perform(context)),
        ('~/landmarks', LaunchConfiguration('landmarks_topic').perform(context)),
        ('~/path', LaunchConfiguration('path_topic').perform(context)),
        # Transitional custom range-to-base (still subscribed in standard mode).
        ('Rangetobase_topic', LaunchConfiguration('rangetobase_topic').perform(context)),
        ('Robotstate_topic', LaunchConfiguration('robotstate_topic').perform(context)),
    ]

    node = ComposableNode(
        package='localslam',
        plugin='localslam::EKFslam',
        name='EKFslam',
        parameters=[params],
        remappings=remappings,
    )
    container = ComposableNodeContainer(
        name='localslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[node],
        output='screen',
    )
    return [container]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='YAML with ros__parameters (default: share/localslam/config/localslam_params.yaml)'),
        DeclareLaunchArgument('sensor_input_mode', default_value='standard'),
        DeclareLaunchArgument(
            'altitude_input_mode',
            default_value='',
            description="derived_altitude|fluid_pressure. Empty = YAML default."),
        DeclareLaunchArgument('pressure_reference_policy', default_value=''),
        DeclareLaunchArgument('pressure_t0_k', default_value=''),
        DeclareLaunchArgument('pressure_min_pa', default_value=''),
        DeclareLaunchArgument('pressure_max_pa', default_value=''),
        DeclareLaunchArgument('pressure_z_lever_arm_m', default_value=''),
        DeclareLaunchArgument('pressure_expected_frame_id', default_value=''),
        DeclareLaunchArgument('pressure_waiting_bound', default_value=''),
        DeclareLaunchArgument(
            'calibration_source',
            default_value='',
            description="yaml|camera_info. Empty = YAML default (yaml)."),
        DeclareLaunchArgument(
            'camera_info_image_path',
            default_value='',
            description="raw|rectified when Calibration_source=camera_info. Empty = YAML."),
        DeclareLaunchArgument(
            'camera_info_expected_frame_id',
            default_value='',
            description='Optional optical frame_id check. Empty = YAML / no override.'),
        DeclareLaunchArgument(
            'camera_info_qos_reliability',
            default_value='',
            description="reliable|best_effort for ~/camera_info (durability stays volatile)."),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'visual_update',
            default_value='',
            description='Override Visual_update (true/false). Empty = YAML.'),
        DeclareLaunchArgument(
            'ground_station_range_update',
            default_value='',
            description='Override Ground_station_range_update. Empty = YAML.'),
        DeclareLaunchArgument('odometry_frame_id', default_value=''),
        DeclareLaunchArgument('odometry_child_frame_id', default_value=''),
        DeclareLaunchArgument('sigma_att_update', default_value=''),
        DeclareLaunchArgument('sigma_spd_update', default_value=''),
        DeclareLaunchArgument('sigma_alt_update', default_value=''),
        DeclareLaunchArgument('sigma_cam_uv', default_value=''),
        DeclareLaunchArgument('sigma_w', default_value=''),
        DeclareLaunchArgument('sigma_a', default_value=''),
        # Remappable I/O (defaults match private-name topics under /EKFslam/...).
        DeclareLaunchArgument('image_topic', default_value='/EKFslam/image'),
        DeclareLaunchArgument('camera_info_topic', default_value='/EKFslam/camera_info'),
        DeclareLaunchArgument('imu_topic', default_value='/EKFslam/imu'),
        DeclareLaunchArgument('body_velocity_topic', default_value='/EKFslam/body_velocity'),
        DeclareLaunchArgument('downward_range_topic', default_value='/EKFslam/downward_range'),
        DeclareLaunchArgument('altitude_topic', default_value='/EKFslam/altitude'),
        DeclareLaunchArgument('pressure_topic', default_value='/EKFslam/pressure'),
        DeclareLaunchArgument('odometry_topic', default_value='/EKFslam/odometry'),
        DeclareLaunchArgument('landmarks_topic', default_value='/EKFslam/landmarks'),
        DeclareLaunchArgument('path_topic', default_value='/EKFslam/path'),
        DeclareLaunchArgument('rangetobase_topic', default_value='/Rangetobase_topic'),
        DeclareLaunchArgument('robotstate_topic', default_value='/Robotstate_topic'),
        OpaqueFunction(function=_build),
    ])
