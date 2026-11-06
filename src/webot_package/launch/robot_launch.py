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
    cmd.extend([
        driver_exec,
        '--ros-args',
        '-p',
        'robot_description:=' + robot_description_path,
        '-p',
        'use_sim_time:=true',
    ])

    # Match WebotsController: WEBOTS_HOME -> driver package prefix so the
    # controller library loads from the ROS install, not /usr/local/webots.
    # cwd must be writable: MavicDriver writes logfile_*.csv relative to cwd
    # unless WEBOT_DRIVER_LOG_DIR is set (driver resolves under ~/.ros/...).
    log_dir = os.path.join(os.path.expanduser('~'), '.ros', 'webot_package_logs')
    os.makedirs(log_dir, exist_ok=True)
    webot_robot_driver = ExecuteProcess(
        cmd=cmd,
        output='screen',
        name='webots_controller_webot_robot',
        respawn=True,
        cwd=log_dir,
        additional_env={
            'WEBOTS_HOME': driver_prefix,
            'WEBOT_DRIVER_LOG_DIR': log_dir,
        },
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='mars.wbt',
            description='World file under webot_package/worlds',
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
