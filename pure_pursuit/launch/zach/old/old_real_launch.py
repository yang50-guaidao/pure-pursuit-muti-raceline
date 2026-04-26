import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('pure_pursuit')

    config = os.path.join(pkg_share, 'config', 'zach_params', 'real_params_old_10laps.yaml')
    # csv_path = os.path.join(pkg_share, 'waypoints', 'race', 'race1_v9_c.csv') 
    csv_path = os.path.join(pkg_share, 'waypoints', 'race', 'race1_v17_2_5.csv') 
    # csv_path = os.path.join(pkg_share, 'waypoints', 'race', 'race1_v6_b_consistent.csv') 

    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            parameters=[config, {'csv_path': csv_path}],
            output='screen'
        )
    ])