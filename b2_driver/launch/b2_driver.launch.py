# b2_driver/launch/b2_driver.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    composable_nodes = []

    composable_node = ComposableNode(
        package='b2_driver',
        plugin='b2_driver::B2Driver',
        name='b2_driver',
        namespace='',
        remappings=[('/cmd_vel', '/b2/cmd_vel')],

    )
    composable_nodes.append(composable_node)

    container = ComposableNodeContainer(
        name='b2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
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
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.006,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(container)
    ld.add_action(pointcloud_to_laserscan_node)

    return ld
