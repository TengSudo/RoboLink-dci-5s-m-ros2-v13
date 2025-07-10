import os

os.environ["IGN_GAZEBO_RESOURCE_PATH"] = (
    os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "") +
    ":/home/tengsudo/cobot_ws/install/cobot_description/share"
)

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, conditions
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'cobot_description'
    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'cobot_description.urdf'
    )

    # Đọc nội dung URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
            DeclareLaunchArgument( # Khai báo argument
            'use_gui', # Tên argument
            default_value='True', # Mặc định hiển thị GUI
            description='Enable Joint State Publisher GUI' # Mô tả
        ),
        # Node phát trạng thái robot
        Node(
            package='robot_state_publisher', # Sử dụng robot_state_publisher
            executable='robot_state_publisher', # Sử dụng robot_state_publisher
            output='screen', # Hiển thị output
            parameters=[{'robot_description': robot_desc}] # Truyền URDF vào robot_state_publisher
        ),

        Node(
            package='joint_state_publisher_gui', # Sử dụng joint_state_publisher_gui
            executable='joint_state_publisher_gui', # Sử dụng joint_state_publisher_gui
            condition=launch.conditions.IfCondition(LaunchConfiguration('use_gui')) # Hiển thị GUI của Joint State Publisher
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["position_controllers", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        # Chạy Ignition Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '/usr/share/ignition/ignition-gazebo6/worlds/empty.sdf'],
            output='screen'
        ),

        # Spawn robot vào Gazebo
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', 'cobot',
                '-file', urdf_file
            ],
            output='screen'
        ),
    ])
