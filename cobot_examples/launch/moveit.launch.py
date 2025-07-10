from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node

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

    realsense_driver_node = Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_camera',
            output='screen',
            parameters=[
                {'enable_depth': True},
                {'enable_color': True},
                {'align_depth': True},
                {'pointcloud.enable': True} #/camera/realsense_camera/depth/color/points
            ]
    )

    return LaunchDescription([
        generate_demo_launch(moveit_config),
        robot_connection_node,
        robot_driver_node,
        #realsense_driver_node
    ])
