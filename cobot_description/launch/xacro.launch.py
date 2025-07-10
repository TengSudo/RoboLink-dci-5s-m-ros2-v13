import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import subprocess


def generate_launch_description():
    pkg_path = get_package_share_directory('cobot_description')
    xacro_path = os.path.join(pkg_path, 'urdf', 'cobot_description.xacro')

    robot_description = subprocess.check_output(['xacro', xacro_path]).decode()

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('cobot_description'),
        'rviz',
        'platform.rviz'
    ])
    
    return LaunchDescription([
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
    ]
    )
