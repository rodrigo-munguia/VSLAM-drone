"""Optional RViz2 viewer for the integrated experiment.

Displays public /odometry, /EKFslam/path, /ground_truth/path, and
/EKFslam/landmarks with Fixed Frame = odom. Path pose axes are off by
default. Does not show Plot2 image overlays or custom stats.

Requires the system `rviz2` package. Estimator-only builds do not depend on it.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('ekf_monoslam_bringup')
    cfg = os.path.join(share, 'config', 'quad_experiment.rviz')
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='ekf_experiment_rviz',
            output='screen',
            arguments=['-d', cfg],
        ),
    ])
