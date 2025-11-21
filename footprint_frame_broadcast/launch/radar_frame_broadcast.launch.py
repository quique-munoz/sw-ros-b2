from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='footprint_frame_broadcast',
            executable='footprint_frame_broadcast_node',
            name='footprint_frame_broadcast',
            parameters=[{
                'source_frame': 'base_footprint',
                'target_frame': 'radar',
                'output_parent_frame': 'base_footprint',
                'output_child_frame': 'radar_flat',
                'output_frame_height': 0.0,
                'yaw_offset': 0.0,
                'publish_rate': 50.0,
                'time_offset': 0.5
            }],
            output='screen'
        )
    ])
