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
            'verbose': True,
            'input_topic': '/zed/zed_node/point_cloud/cloud_registered',
            'output_topic': '/pointcloud_accumulated',
            'use_sim_time': True,
            'max_buffer_size': 50,
            'time_window_seconds': 2.0,
            'output_frequency': 10.0,
            'target_frame': 'orbit_frame',
            'fixed_frame': 'map',
            'tf_timeout_ms': 0,
        }]
    )

    return LaunchDescription([
        pointcloud_accumulator_node,
    ]) 