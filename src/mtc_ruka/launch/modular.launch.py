from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ruka", package_name="ruka")
        .robot_description(file_path="config/ruka.urdf.xacro")
        .to_moveit_configs()
    )

    modular_task = Node(
        package="moveit_task_constructor_demo",
        executable="modular",
        output="screen",
        parameters=[
            moveit_config.joint_limits,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([modular_task])
