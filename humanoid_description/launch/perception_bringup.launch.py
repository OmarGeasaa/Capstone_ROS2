import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Path Setup
    pkg_description = get_package_share_directory('humanoid_description')
    pkg_astra = get_package_share_directory('astra_camera')
    pkg_rplidar = get_package_share_directory('sllidar_ros2')

    # Path to your URDF and RViz config
    urdf_file = os.path.join(pkg_description, 'urdf', 'head.urdf')
    rviz_config = os.path.join(pkg_description, 'rviz', 'perception_view.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 2. Nodes & Includes
    
    # Robot State Publisher (The TF Brain)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # RPLiDAR A1 Driver
    # Note: Ensure your serial_port matches (usually /dev/ttyUSB0)
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rplidar, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={'serial_port': '/dev/ttyUSB0'}.items()
    )

    # Orbbec Astra Pro Driver
    astra_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(pkg_astra, 'launch', 'astra_pro.launch.xml')
	)
    )
    # RViz2 (The Visualizer)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        rplidar_launch,
        astra_launch,
        rviz_node
    ])
