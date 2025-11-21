#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/cmd_vel_teleop',
        description='Input topic for twist messages'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/cmd_vel_stamped',
        description='Output topic for stamped twist messages'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='base_link',
        description='Frame ID for the stamped twist messages'
    )

    # Create the node
    twist_to_stamped_node = Node(
        package='twist_to_stamped',
        executable='twist_to_stamped_node',
        name='twist_to_stamped',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
        }]
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        frame_id_arg,
        twist_to_stamped_node,
    ]) 