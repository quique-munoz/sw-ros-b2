# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim


import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node



def generate_launch_description():


    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam = LaunchConfiguration('slam', default='False')
    map = LaunchConfiguration('map', default='my_map')

    # map_dir = os.path.join(get_package_share_directory('b2_navigation'), 'maps', 'empty.yaml')
    map_dir = PathJoinSubstitution([
        get_package_share_directory('b2_navigation'),
        'maps',
        map,
        [map, TextSubstitution(text='.yaml')]
    ])


    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('b2_navigation'),
            'config',
            'navigation_params.yaml',
            ))

    slam_param_dir = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(
            get_package_share_directory('b2_navigation'),
            'config',
            'mapper_params_online_async.yaml',
        )
    )

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    

    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'slam_params_file',
            default_value=slam_param_dir,
            description='Full path to slam param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'slam',
            default_value='False',
            description='Use SLAM if true'),

        DeclareLaunchArgument(
            'map',
            default_value='my_map',
            description='Map name'),



        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_dir,
                'slam': slam,
                'params_file': param_dir,
                'slam_params_file': slam_param_dir,
                }.items(),
        ),

    ])

