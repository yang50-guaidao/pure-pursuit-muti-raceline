import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('opp_predictor')
    config = os.path.join(pkg_share, 'config', 'opp_predictor_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name', default_value='racetrack_test',
            description='Map name (must have matching .yaml/.pgm in share/opp_predictor/maps/ '
                        'and bound .npy files in share/opp_predictor/csv/<map_name>/)'),
        DeclareLaunchArgument(
            'real_test', default_value='true',
            description='true = real car (PoseStamped pose), false = sim (Odometry pose)'),

        Node(
            package='opp_predictor',
            executable='opp_predictor_node',
            name='opp_predictor_node',
            parameters=[
                config,
                {'map_name': LaunchConfiguration('map_name')},
                {'real_test': LaunchConfiguration('real_test')},
            ],
            output='screen',
        ),
    ])
