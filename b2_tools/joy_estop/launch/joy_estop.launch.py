#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/joy',
        description='Input topic for joy messages'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/e_stop',
        description='Output topic for e_stop messages'
    )
    
    # On a PS5 controller using game_controller_node:
    # - X: 0
    # - Circle: 1
    # - Square: 2
    # - Triangle: 3
    # - Select: 4
    # - PS: 5
    # - Start: 6
    # - L3: 7
    # - R3: 8
    # - L1: 9
    # - R1: 10
    # - Up: 11
    # - Down: 12
    # - Left: 13
    # - Right: 14
    
    estop_button_arg = DeclareLaunchArgument(
        'estop_button',
        default_value='1',
        description='E-stop button'
    )

    reset_button_arg = DeclareLaunchArgument(
        'reset_button',
        default_value='3',
        description='Reset button'
    )

    damp_buttons_arg = DeclareLaunchArgument(
        'damp_buttons',
        default_value='[7, 8]',
        description='Damp buttons'
    )

    stand_down_buttons_arg = DeclareLaunchArgument(
        'stand_down_buttons',
        default_value='[9, 10, 12]',
        description='Stand down buttons'
    )

    stand_up_buttons_arg = DeclareLaunchArgument(
        'stand_up_buttons',
        default_value='[9, 10, 11]',
        description='Stand up buttons'
    )

    # Create the node
    joy_estop_node = Node(
        package='joy_estop',
        executable='joy_estop_node',
        name='joy_estop',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'estop_button': LaunchConfiguration('estop_button'),
            'reset_button': LaunchConfiguration('reset_button'),
            'damp_buttons': LaunchConfiguration('damp_buttons'),
            'stand_down_buttons': LaunchConfiguration('stand_down_buttons'),
            'stand_up_buttons': LaunchConfiguration('stand_up_buttons'),
        }]
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        estop_button_arg,
        reset_button_arg,
        damp_buttons_arg,
        stand_down_buttons_arg,
        stand_up_buttons_arg,
        joy_estop_node,
    ]) 