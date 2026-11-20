#!/usr/bin/env python3
# Copyright 1996-2023 Cyberbotics Ltd.
# Adapted for ROS 2 Jazzy / webots_ros2 R2025a.
#
# Stock WebotsController wraps:
#   webots-controller ... ros2 --ros-args -p robot_description:=...
# which expands to `ros2 run webots_ros2_driver driver`. That fails when
# ros-jazzy-ros2run is not installed (`ros2: invalid choice: 'run'`).
# This launch keeps the official webots-controller IPC setup but passes the
# driver executable path directly so `ros2 run` is not required.

"""Launch Webots Mavic-based webot_robot driver for the EKF-MonoSLAM experiment."""

import os

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_prefix, get_package_share_directory

_INSTALL_HINT = (
    'webots_ros2_driver is required for this launch file.\n'
    'Exact supported install on Ubuntu 24.04 + ROS 2 Jazzy:\n'
    '  wget https://github.com/cyberbotics/webots/releases/download/'
    'R2025a/webots_2025a_amd64.deb\n'
    '  sudo apt install ./webots_2025a_amd64.deb\n'
    '  sudo apt-get install ros-jazzy-webots-ros2\n'
    '  export WEBOTS_HOME=/usr/local/webots\n'
    'See docs/A6_webots_jazzy_bringup.md'
)


def generate_launch_description():
    try:
        from webots_ros2_driver.webots_launcher import WebotsLauncher
        from webots_ros2_driver.utils import controller_protocol, controller_ip_address
    except ImportError as exc:
        raise RuntimeError('{0}\n({1})'.format(_INSTALL_HINT, exc)) from exc

    package_dir = get_package_share_directory('webot_package')
    world = LaunchConfiguration('world')
    robot_description_path = os.path.join(
        package_dir, 'resource', 'webot_robot.urdf')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True,
    )

    driver_prefix = get_package_prefix('webots_ros2_driver')
    webots_controller = os.path.join(
        get_package_share_directory('webots_ros2_driver'),
        'scripts',
        'webots-controller',
    )
    driver_exec = os.path.join(driver_prefix, 'lib', 'webots_ros2_driver', 'driver')

    protocol = controller_protocol()
    ip_address = controller_ip_address() if protocol == 'tcp' else ''
    cmd = [
        webots_controller,
        '--robot-name=webot_robot',
        '--protocol=' + protocol,
        '--port=1234',
    ]
    if ip_address:
        cmd.append('--ip-address=' + ip_address)
    # R5a production: traj XY/Z + tracker XY/Z (defaults in DeclareLaunchArgument).
    traj_v_max = LaunchConfiguration('traj_v_max')
    traj_v_max_xy = LaunchConfiguration('traj_v_max_xy')
    traj_v_max_z = LaunchConfiguration('traj_v_max_z')
    traj_a_max = LaunchConfiguration('traj_a_max')
    traj_j_max = LaunchConfiguration('traj_j_max')
    tracker_v_max_xy = LaunchConfiguration('tracker_v_max_xy')
    tracker_v_max_z = LaunchConfiguration('tracker_v_max_z')
    cmd.extend([
        driver_exec,
        '--ros-args',
        '-p',
        'robot_description:=' + robot_description_path,
        '-p',
        'use_sim_time:=true',
        '-p', ['traj_v_max:=', traj_v_max],
        '-p', ['traj_v_max_xy:=', traj_v_max_xy],
        '-p', ['traj_v_max_z:=', traj_v_max_z],
        '-p', ['traj_a_max:=', traj_a_max],
        '-p', ['traj_j_max:=', traj_j_max],
        '-p', ['tracker_v_max_xy:=', tracker_v_max_xy],
        '-p', ['tracker_v_max_z:=', tracker_v_max_z],
    ])

    # Match WebotsController: WEBOTS_HOME -> driver package prefix so the
    # controller library loads from the ROS install, not /usr/local/webots.
    # cwd must be writable: MavicDriver writes logfile_*.csv relative to cwd
    # unless WEBOT_DRIVER_LOG_DIR is set (driver resolves under ~/.ros/...).
    log_dir = os.path.join(os.path.expanduser('~'), '.ros', 'webot_package_logs')
    os.makedirs(log_dir, exist_ok=True)
    # Env mirrors launch args so MavicDriver (inner mavic_driver node) sees
    # overrides; CLI -p alone does not reach the plugin-created node.
    webot_robot_driver = ExecuteProcess(
        cmd=cmd,
        output='screen',
        name='webots_controller_webot_robot',
        respawn=True,
        cwd=log_dir,
        additional_env={
            'WEBOTS_HOME': driver_prefix,
            'WEBOT_DRIVER_LOG_DIR': log_dir,
            'WEBOT_TRAJ_V_MAX': traj_v_max,
            'WEBOT_TRAJ_V_MAX_XY': traj_v_max_xy,
            'WEBOT_TRAJ_V_MAX_Z': traj_v_max_z,
            'WEBOT_TRAJ_A_MAX': traj_a_max,
            'WEBOT_TRAJ_J_MAX': traj_j_max,
            'WEBOT_TRACKER_V_MAX_XY': tracker_v_max_xy,
            'WEBOT_TRACKER_V_MAX_Z': tracker_v_max_z,
        },
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='mars.wbt',
            description='World file under webot_package/worlds',
        ),
        DeclareLaunchArgument(
            'traj_v_max',
            default_value='5.0',
            description='Legacy isotropic traj vmax [m/s]; prefer traj_v_max_xy/z',
        ),
        DeclareLaunchArgument(
            'traj_v_max_xy',
            default_value='5.0',
            description='A3g production traj XY vmax [m/s]; ≤0 inherits traj_v_max; prior 3.0 via flag',
        ),
        DeclareLaunchArgument(
            'traj_v_max_z',
            default_value='1.5',
            description='Production traj Z vmax [m/s]; ≤0 inherits traj_v_max',
        ),
        DeclareLaunchArgument(
            'traj_a_max',
            default_value='0.8',
            description='Trajectory amax [m/s²] (production 0.8)',
        ),
        DeclareLaunchArgument(
            'traj_j_max',
            default_value='2.0',
            description='Trajectory jmax [m/s³] (production 2.0)',
        ),
        DeclareLaunchArgument(
            'tracker_v_max_xy',
            default_value='5.5',
            description='Tracker horizontal v_sp sat [m/s] (A3g 5.5; prior 3.5 via flag)',
        ),
        DeclareLaunchArgument(
            'tracker_v_max_z',
            default_value='1.5',
            description='Tracker vertical v_sp sat [m/s] (production 1.5)',
        ),
        webots,
        webots._supervisor,
        webot_robot_driver,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=Shutdown())],
            )
        ),
    ])
