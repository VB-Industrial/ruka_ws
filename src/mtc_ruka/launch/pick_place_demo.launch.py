from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
  
    
    moveit_config = (
        MoveItConfigsBuilder("ruka", package_name="ruka")
        .robot_description(
            file_path="config/ruka.urdf.xacro",
            # mappings={
            #     "ros2_control_hardware_type": LaunchConfiguration(
            #         "ros2_control_hardware_type"
            #     )
            # },
        )
        .robot_description_semantic(file_path="config/ruka.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_dict()
    )
    
    # MTC Demo node
    pick_place_demo = Node(
        package="mtc_ruka",
        executable="mtc_ruka",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])
