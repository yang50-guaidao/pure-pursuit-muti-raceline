import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('pure_pursuit_multi')

    config = os.path.join(pkg_share, 'config', 'zach_params', 'real_params.yaml')

    csv_path = os.path.join(pkg_share, 'waypoints', 'race', 'apr14_3.csv')

    return LaunchDescription([
        Node(
            package='pure_pursuit_multi',
            executable='pure_pursuit_multi_node',
            name='pure_pursuit_multi_node',
            parameters=[config, {'csv_path': csv_path}],
            output='screen'
        )
    ])