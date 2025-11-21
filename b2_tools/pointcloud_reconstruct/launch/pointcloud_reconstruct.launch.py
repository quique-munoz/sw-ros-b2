#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = False
    
    # Create the node
    pointcloud_reconstruct_node = Node(
        package='pointcloud_reconstruct',
        executable='pointcloud_reconstruct_node',
        name='pointcloud_reconstruct',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'verbose': True,
            'input_topic': '/zed/zed_node/point_cloud/cloud_registered',
            'output_topic': '/pointcloud_reconstructed',
            'centroid_topic': '/reconstruction/centroid',
            'publish_rate': 5.0,
            'fixed_frame': 'map',
            'tf_timeout_ms': 50,
            'filter_voxel_grid_size': 0.01,
            'visualization_voxel_grid_size': 0.005,
            'max_depth': 1.0,
            'max_accumulated_points': 2000000,
        }]
    )

    # Create the orbit node
    orbit_frame_broadcast_node = Node(
            package='orbit_frame_broadcast',
            executable='orbit_frame_broadcast_node',
            name='orbit_frame_broadcast',
            parameters=[{
                'use_sim_time': use_sim_time,
                'parent_frame': 'map',
                'child_frame': 'orbit_frame',
                'orbit_radius': 2.0,
                'orbit_center_x': 0.9,
                'orbit_center_y': 0.6,
                'orbit_center_z': 0.3,
                'orbit_speed': 0.4,
                'orbit_pitch': 0.0,
                'publish_rate': 50.0,
                'time_offset': 0.1,
                'verbose': False
            }],
            output='screen'
        )

    # Rviz
    rviz_config_file = os.path.join(get_package_share_directory('pointcloud_reconstruct'), 'rviz', 'rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        pointcloud_reconstruct_node,
        orbit_frame_broadcast_node,
        rviz_node
    ]) 
