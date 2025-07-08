from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("igus_DCi_5S_M", package_name="cobot_moveit_config").trajectory_execution(file_path="config/moveit_controllers.yaml").to_moveit_configs()
    
    robot_connection_node = Node(
        package='cobot_hardware',
        executable='robot_connection',
        output='screen',
        name='robot_connection',
        remappings=[
            ('robot/actual_joint_states', '/joint_states')
        ]
    )

    robot_driver_node = Node(
        package='cobot_controller',
        executable='real_arm_controller',
        name='real_arm_controller',
        output='screen'
    )

    return LaunchDescription([
        generate_demo_launch(moveit_config),
        robot_connection_node,
        robot_driver_node
    ])
