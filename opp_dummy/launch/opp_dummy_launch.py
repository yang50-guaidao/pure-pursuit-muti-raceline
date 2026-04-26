import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('opp_dummy')
    config = os.path.join(pkg_share, 'config', 'opp_dummy_params.yaml')

    return LaunchDescription([
        Node(
            package='opp_dummy',
            executable='opp_dummy_node',
            name='opp_dummy_node',
            parameters=[config],
            output='screen',
        ),
    ])
