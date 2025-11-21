from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='footprint_frame_broadcast',
            executable='footprint_frame_broadcast_node',
            name='footprint_frame_broadcast',
            parameters=[{
                'source_frame': 'odom',
                'target_frame': 'base_link',
                'output_parent_frame': 'odom',
                'output_child_frame': 'base_footprint',
                'output_frame_height': 0.35,
                'yaw_offset': 0.0,
                'publish_rate': 50.0,
                'time_offset': 0.5
            }],
            output='screen'
        )
    ])
