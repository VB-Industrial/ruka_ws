# from launch import LaunchDescription
# from launch_ros.actions import Node
# from moveit_configs_utils import MoveItConfigsBuilder


# def generate_launch_description():
#     moveit_config = (
#         MoveItConfigsBuilder("ruka", package_name="ruka")
#         .robot_description(file_path="config/ruka.urdf.xacro")
#         .robot_description_semantic(file_path="config/ruka.srdf")
#         .to_moveit_configs()
#     )

#     cartesian_task = Node(
#         package="moveit_task_constructor_demo",
#         executable="cartesian",
#         output="screen",
#         parameters=[
#             moveit_config.joint_limits,
#             moveit_config.robot_description,
#             moveit_config.robot_description_semantic,
#             moveit_config.robot_description_kinematics,
#         ],
#     )

#     return LaunchDescription([cartesian_task])
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ruka", package_name="ruka_end_eff")
        .robot_description(
            file_path="config/ruka.urdf.xacro",
           
        )
        .robot_description_semantic(file_path="config/ruka.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )

    # MTC Demo node
    pick_place_demo = Node(
        package="mtc_ruka",
        executable="flag",
        output="screen",
        parameters=[
            moveit_config.joint_limits,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([pick_place_demo])