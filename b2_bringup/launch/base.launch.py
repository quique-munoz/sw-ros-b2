# ROS2 Launch file

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    lidar = LaunchConfiguration('lidar')
    realsense = LaunchConfiguration('realsense')
    rviz = LaunchConfiguration('rviz')
    zed = LaunchConfiguration('zed')

    declare_lidar_cmd = DeclareLaunchArgument(
        'lidar',
        default_value='False',
        description='Launch RoboSense RS16 lidar driver'
    )

    declare_realsense_cmd = DeclareLaunchArgument(
        'realsense',
        default_value='False',
        description='Launch realsense driver'
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Launch rviz'
    )

    declare_zed_cmd = DeclareLaunchArgument(
        'zed',
        default_value='False',
        description='Launch zed driver'
    )

    robot_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('b2_description'),
            'launch/'), 'robot.launch.py'])
    )

    driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('b2_driver'),
            'launch/'), 'b2_driver.launch.py'])
    )

    # Create lidar command only if package is available
    try:
        rslidar_sdk_path = get_package_share_directory('rslidar_sdk')
    except:
        rslidar_sdk_path = "/error/rslidar_sdk/not_found"

    lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            rslidar_sdk_path,
            'launch/'), 'start.py']),
        condition=IfCondition(PythonExpression([lidar]))
    )

    # Create realsense command only if package is available
    try:
        realsense_path = get_package_share_directory('realsense2_camera')
    except:
        realsense_path = "/error/realsense2_camera/not_found"

    realsense_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            realsense_path,
                'launch/'), 'rs_launch.py']),
            launch_arguments={
                'camera_namespace': "camera",
                'camera_name': "realsense"        
            }.items(),
            condition=IfCondition(PythonExpression([realsense]))
        )

    # Create zed command only if package is available
    try:
        zed_path = get_package_share_directory('zed_wrapper')
    except:
        zed_path = "/error/zed_wrapper/not_found"

    zed_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            zed_path, 'launch/'), 'zed_camera.launch.py']),
            launch_arguments={
                'camera_namespace': "camera",
                'camera_name': "zed",
                'camera_model': "zed2i"
            }.items(),
            condition=IfCondition(PythonExpression([zed]))
        )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('b2_rviz'),
            'launch/'), 'rviz.launch.py']),
        condition=IfCondition(PythonExpression([rviz]))
    )

    ld = LaunchDescription()
    ld.add_action(declare_lidar_cmd)
    # ld.add_action(declare_realsense_cmd)
    # ld.add_action(declare_rviz_cmd)
    ld.add_action(robot_description_cmd)
    # ld.add_action(declare_zed_cmd)
    ld.add_action(lidar_cmd)
    # ld.add_action(realsense_cmd)
    ld.add_action(driver_cmd)
    # ld.add_action(zed_cmd)
    ld.add_action(rviz_cmd)

    return ld
