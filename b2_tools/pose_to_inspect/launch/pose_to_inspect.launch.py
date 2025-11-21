#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/euler',
        description='Output topic for euler messages'
    )

    # Create the node
    pose_to_inspect_node = Node(
        package='pose_to_inspect',
        executable='pose_to_inspect_node',
        name='pose_to_inspect',
        output='screen',
        parameters=[{
            'verbose': True,
            'output_topic': LaunchConfiguration('output_topic'),
            'velocity': 0.1,    # rad/s
            'control_dt_ms': 100, # ms
        }]
    )

    return LaunchDescription([
        output_topic_arg,
        pose_to_inspect_node,
    ]) 