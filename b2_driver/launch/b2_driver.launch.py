# b2_driver/launch/b2_driver.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Nodo principal del driver del B2
    b2_driver_node = Node(
        package='b2_driver',
        executable='b2_driver_node',   
        name='b2_driver',
        namespace='',
        output='screen',
        # remappings=[('cmd_vel', '/b2/cmd_vel')],
    )

    # Nodo que convierte pointcloud -> laserscan
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        namespace='',
        output='screen',
        remappings=[('cloud_in', '/pointcloud')],  
        parameters=[{
            'target_frame': 'radar_flat',
            'transform_tolerance': 0.01,
            # radar_flat con Z hacia abajo: negativo es por encima del sensor
            'min_height': -0.1,
            'max_height': 0.1,
            'range_min': 0.0,
            'range_max': 50.0,
            'angle_min': -1.57,
            'angle_max': 1.57,
            'angle_increment': 0.006,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(b2_driver_node)
    ld.add_action(pointcloud_to_laserscan_node)

    return ld
