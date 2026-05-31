from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- LEFT LEG NODES ---
    left_ankle_ik = Node(
        package='biped_kinematics',
        executable='ankle_ik_node',
        name='left_ankle_ik_node',
        parameters=[{'leg_prefix': 'left'}]
    )
    left_ankle_fk = Node(
        package='biped_kinematics',
        executable='ankle_fk_node',
        name='left_ankle_fk_node',
        parameters=[{'leg_prefix': 'left'}]
    )
    left_hip_ik = Node(
        package='biped_kinematics',
        executable='hip_ik_node',
        name='left_hip_ik_node',
        parameters=[{'leg_prefix': 'left'}]
    )
    left_hip_fk = Node(
        package='biped_kinematics',
        executable='hip_fk_node',
        name='left_hip_fk_node',
        parameters=[{'leg_prefix': 'left'}]
    )

    # --- RIGHT LEG NODES ---
    right_ankle_ik = Node(
        package='biped_kinematics',
        executable='ankle_ik_node',
        name='right_ankle_ik_node',
        parameters=[{'leg_prefix': 'right'}]
    )
    right_ankle_fk = Node(
        package='biped_kinematics',
        executable='ankle_fk_node',
        name='right_ankle_fk_node',
        parameters=[{'leg_prefix': 'right'}]
    )
    right_hip_ik = Node(
        package='biped_kinematics',
        executable='hip_ik_node',
        name='right_hip_ik_node',
        parameters=[{'leg_prefix': 'right'}]
    )
    right_hip_fk = Node(
        package='biped_kinematics',
        executable='hip_fk_node',
        name='right_hip_fk_node',
        parameters=[{'leg_prefix': 'right'}]
    )

    return LaunchDescription([
        # To test ONLY the left leg, simply comment out the right leg nodes below!
        left_ankle_ik,
        left_ankle_fk,
        left_hip_ik,
        left_hip_fk,
        
        right_ankle_ik,
        right_ankle_fk,
        right_hip_ik,
        right_hip_fk
    ])
