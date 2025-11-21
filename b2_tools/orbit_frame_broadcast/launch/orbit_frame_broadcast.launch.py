from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        Node(
            package='orbit_frame_broadcast',
            executable='orbit_frame_broadcast_node',
            name='orbit_frame_broadcast',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
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
    ])
