import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    topic_arg = launch.actions.DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_teleop')
    device_arg = launch.actions.DeclareLaunchArgument('joy_dev', default_value='0')
    teleop_config_arg = launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
                            launch.substitutions.TextSubstitution(text=os.path.join(
                                get_package_share_directory('b2_bringup'), 'config', 'teleop_config.yaml'))])

    joy_node = launch_ros.actions.Node(
        package='joy', executable='game_controller_node', name='game_controller_node',
        parameters=[{
            'device_id': joy_dev,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }])
        
    teleop_node = launch_ros.actions.Node(
        package='teleop_twist_joy', executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_filepath],
        remappings={('/cmd_vel', launch.substitutions.LaunchConfiguration('cmd_vel_topic'))},
        )

    return launch.LaunchDescription([
        topic_arg,
        device_arg,
        teleop_config_arg,
        
        joy_node,
        teleop_node,
    ])
