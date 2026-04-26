import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('pure_pursuit')

    config = os.path.join(pkg_share, 'config', 'zach_params', 'boost_params.yaml')
    csv_path = os.path.join(pkg_share, 'waypoints', 'race', 'race1_v17_2_5_brakemode.csv')

    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='pure_pursuit_boost',
            name='pure_pursuit_boost_node',
            parameters=[config, {'csv_path': csv_path}],
            output='screen'
        )
    ])