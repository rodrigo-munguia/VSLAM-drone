"""Optional A4 RViz2 view for odometry, landmarks, and path.

Requires rviz2 installed on the system. Estimator / headless builds do not
depend on this launch file. Does not install system packages.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('ekf_monoslam_bringup')
    cfg = os.path.join(share, 'config', 'a4_localslam.rviz')
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='a4_rviz',
            output='screen',
            arguments=['-d', cfg],
        ),
    ])
