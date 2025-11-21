# ROS2 launch file

from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os



def generate_launch_description():
    pkg_share_pointcloud_accumulator = FindPackageShare(package='pointcloud_accumulator').find('pointcloud_accumulator')    
    pkg_share_b2 = FindPackageShare(package='b2_bringup').find('b2_bringup')    
    
    
    # Twist mux
    twist_mux_file_dir = os.path.join(get_package_share_directory('b2_bringup'), 'config', 'twist_mux_params.yaml')

    twist_mux_node = Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            remappings={('/cmd_vel_out', '/b2/cmd_vel')},
            parameters=[twist_mux_file_dir],
        )

    # Joy estop
    joy_estop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('joy_estop'),
                'launch/joy_estop.launch.py'
            )
        )
    )

    # b2
    # This launches the b2, enables control by cmd_vel, and starts the pointcloud-to-scan node
    b2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource( 
            os.path.join(
                pkg_share_b2,
                "launch/base.launch.py",
            )
        ),
        launch_arguments={
            'lidar': "False",      
            'realsense': "False",   
            'rviz': "False",
            'zed': "False"        
        }.items()
    )

    # Base footprint frame broadcast
    base_footprint_broadcast_node = Node(
            package='footprint_frame_broadcast',
            executable='footprint_frame_broadcast_node',
            name='base_footprint_frame_broadcast',
            parameters=[{
                'source_frame': 'odom',
                'target_frame': 'base_link',
                'output_parent_frame': 'odom',
                'output_child_frame': 'base_footprint',
                'output_frame_height': 0.35,
                'yaw_offset': 0.0,
                'publish_rate': 50.0,
                'time_offset': 0.05
            }],
            output='screen'
        )
    
    # Radar footprint frame broadcast
    radar_frame_broadcast_node = Node(
            package='footprint_frame_broadcast',
            executable='footprint_frame_broadcast_node',
            name='radar_footprint_frame_broadcast',
            parameters=[{
                'source_frame': 'base_footprint',
                'target_frame': 'radar',
                'output_parent_frame': 'base_footprint',
                'output_child_frame': 'radar_flat',
                'output_frame_height': 0.0,
                'yaw_offset': -3.14159,
                'publish_rate': 50.0,
                'time_offset': 0.05
            }],
            output='screen'
        )
    
    # Pointcloud accumulator
    pointcloud_accumulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource( 
            os.path.join(
                pkg_share_pointcloud_accumulator,
                "launch/pointcloud_accumulator.launch.py",
            )
        )
    )

    # Pointcloud to laserscan of the accumulated pointcloud
    pointclod_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_accumulated',
        namespace='',
        output='screen',
        remappings=[('/cloud_in', '/pointcloud_accumulated'),
                    ('/scan', '/scan_accumulated')
        ],
        parameters=[{
                'target_frame': 'radar_flat',
                'transform_tolerance': 0.1,
                'min_height': -0.25, # radar_flat tf is pointing Z-up, so positive is above the sensor
                'max_height': 0.2,  # and negative is below the sensor
                'range_min': 0.0,
                'range_max': 50.0,
                'angle_min': -1.57,
                'angle_max': 1.57, 
                'angle_increment': 0.0021,  # This parameter can break SLAM toolbox, modify if it fails
                'scan_time': 0.1, # time in seconds (1/30 Hz)
            }],
    )

    # Camera stream to ROS2
    b2_eth_device = os.getenv('GO2_ETH_DEVICE', 'eth0')  # Default to eth0 if not set
    gstream_config_string = 'gst-launch-1.0 udpsrc address=230.1.1.1 port=1720 multicast-iface=' + b2_eth_device + ' ! queue !  application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert'
    gscam_node = Node(
        package='gscam',
        executable='gscam_node',
        output='screen',
        
        parameters=[{
            'gscam_config': gstream_config_string,
            'frame_id': 'Head_upper',
            # 'image_encoding': 'rgb8',
            # 'camera_info_url': 'file://' + os.path.join(pkg_share_b2, 'config', 'camera_info.yaml'),
            # 'camera_name': 'internal_camera',
            # 'sync_sink': False,
        }],
        remappings=[    # Remaps to rename to internal camera
            ('/camera/image_raw', '/camera/internal_camera/image_raw'),
            ('/camera/camera_info', '/camera/internal_camera/camera_info'),
            ('/camera/image_raw/compressed', '/camera/internal_camera/image_raw/compressed'),
            ('/camera/image_raw/theora', '/camera/internal_camera/image_raw/theora'),
            ('/camera/image_raw/compressedDepth', '/camera/internal_camera/image_raw/compressedDepth'),
        ]
    )

    # Create launch description
    return LaunchDescription([
        twist_mux_node,
        joy_estop_launch,
        b2_launch,
        base_footprint_broadcast_node,
        radar_frame_broadcast_node,
        pointcloud_accumulator_launch,
        pointclod_to_laserscan_node,
        # gscam_node,
    ])
