#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    ddsm_node = Node(
        package='vision_motion',
        executable='run_ddsm',
        parameters=[],
        arguments=[],
        output="screen",
    )

    opencm_node = Node(
        package='vision_motion',
        executable='run_cm',
        parameters=[],
        arguments=[],
        output="screen",
    )

    # create and return launch description object
    return LaunchDescription(
        [
            ddsm_node,
            opencm_node,
        ]
    )