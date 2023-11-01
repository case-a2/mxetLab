#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # ...

    # DECLARE Gazebo WORLD file (from conveyorbelt_gazebo package)
    conveyorbelt_gazebo = os.path.join(
        get_package_share_directory('conveyorbelt_gazebo'),
        'launch',
        'conveyorbelt.launch.py')

    # DECLARE Gazebo LAUNCH file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([conveyorbelt_gazebo]))

    # ADD LAUNCH ACTION FOR THE ITEM SPAWNER NODE (from mxet_ros2_labs package)
    # item_spawner_node = Node(
    #     package='mxet_ros2_labs',  
    #     executable='item_spawner.py',  # python script
    #     output='screen',
    # )

    return LaunchDescription([
        gazebo,  # Include the Gazebo launch
        # item_spawner_node,  # Add the item spawning node to the launch
    ])
