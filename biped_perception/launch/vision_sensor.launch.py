import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource 
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Trigger the Astra Camera XML Launch File
    astra_pkg_dir = get_package_share_directory('astra_camera')
    astra_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(astra_pkg_dir, 'launch', 'astra_pro.launch.xml') 
        )
    )

    # 2. Trigger the RPLidar Node Directly
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyUSB0'},
            {'frame_id': 'lidar_link'},  
            {'angle_compensate': True},
            {'scan_mode': 'Standard'}
        ]
    )

    return LaunchDescription([
        astra_launch,
        lidar_node
    ])
