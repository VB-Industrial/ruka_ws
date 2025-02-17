import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder




def generate_launch_description():

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="mtc.rviz",
        description="RViz configuration file",
    )

    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )          

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
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("mtc_ruka") + "/config/mtc.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # # Static TF
    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    # )

    # # Publish TF
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="both",
    #     parameters=[moveit_config.robot_description],
    # )

    # # ros2_control using FakeSystem as hardware
    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("ruka"),
    #     "config",
    #     "ros2_controllers.yaml",
    # )
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[ros2_controllers_path],
    #     remappings=[
    #         ("/controller_manager/robot_description", "/robot_description"),
    #     ],
    #     output="screen",
    # )

    # Load controllers
    # load_controllers = []
    # for controller in [
    #     "panda_arm_controller",
    #     "panda_hand_controller",
    #     "joint_state_broadcaster",
    # ]:
    #     load_controllers += [
    #         ExecuteProcess(
    #             cmd=["ros2 run controller_manager spawner {}".format(controller)],
    #             shell=True,
    #             output="screen",
    #         )
    #     ]
    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "joint_state_broadcaster",
    #         "--controller-manager",
    #         "/controller_manager",
    #     ],
    # )

    # ruka_arm_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["ruka_arm_controller", "-c", "/controller_manager"],
    # )
    # sensor_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["is_broadcaster", "--controller-manager", "/controller_manager"],
    # )
    db_config = LaunchConfiguration("db")
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
        condition=IfCondition(db_config),
    )

    return LaunchDescription(
       [ 
            rviz_node,
            db_arg,
            # static_tf,
            # robot_state_publisher,
            run_move_group_node,
            # ros2_control_node,
            # joint_state_broadcaster_spawner,
            # ruka_arm_controller_spawner,
            # sensor_spawner
            mongodb_server_node
       ]
    )
