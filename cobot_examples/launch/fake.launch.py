from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("igus_DCi_5S_M", package_name="cobot_moveit_config") \
        .trajectory_execution(file_path="config/moveit_controllers.yaml") \
        .to_moveit_configs()

    fake_arm_controller_node = Node(
        package='cobot_controller',
        executable='fake_arm_controller',
        name='fake_arm_controller',
        output='screen',
        parameters=[{
            'joint_topic': '/joint_angles'
        }],
        respawn=True,
        respawn_delay=1.0
    )
    return LaunchDescription([
        generate_demo_launch(moveit_config),
        fake_arm_controller_node
    ])
