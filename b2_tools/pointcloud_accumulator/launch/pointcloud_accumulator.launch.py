#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Create the node
    pointcloud_accumulator_node = Node(
        package='pointcloud_accumulator',
        executable='pointcloud_accumulator_node',
        name='pointcloud_accumulator',
        output='screen',
        parameters=[{
            'verbose': False,
            'input_topic': '/pointcloud',
            'output_topic': '/pointcloud_accumulated',
            'use_sim_time': False,
            'max_buffer_size': 50,
            'time_window_seconds': 0.5,
            'output_frequency': 10.0,
            'target_frame': 'radar_flat',
            'fixed_frame': 'odom',
            'tf_timeout_ms': 10,
        }]
    )

    return LaunchDescription([
        pointcloud_accumulator_node,
    ]) 