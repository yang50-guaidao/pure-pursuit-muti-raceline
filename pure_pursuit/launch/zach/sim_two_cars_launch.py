"""Sim launch with both ego (pure_pursuit_multi) and opp (opp_dummy) running.

Assumes f1tenth_gym_ros is started separately with num_agent: 2.
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pp_share = get_package_share_directory('pure_pursuit_multi')
    opp_share = get_package_share_directory('opp_dummy')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pp_share, 'launch', 'zach', 'sim_launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(opp_share, 'launch', 'opp_dummy_launch.py')
            )
        ),
    ])
