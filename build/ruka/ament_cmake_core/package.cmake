set(_AMENT_PACKAGE_NAME "ruka")
set(ruka_VERSION "0.3.0")
set(ruka_MAINTAINER "N <nata@voltbro.com>")
set(ruka_BUILD_DEPENDS "control_msgs" "controller_interface" "hardware_interface" "gz_ros2_control" "kdl_parser" "pluginlib" "rclcpp_lifecycle" "rclcpp" "rcpputils" "realtime_tools" "trajectory_msgs" "gz_sim_vendor" "gz_plugin_vendor")
set(ruka_BUILDTOOL_DEPENDS "ament_cmake")
set(ruka_BUILD_EXPORT_DEPENDS "control_msgs" "controller_interface" "hardware_interface" "gz_ros2_control" "kdl_parser" "pluginlib" "rclcpp_lifecycle" "rclcpp" "rcpputils" "realtime_tools" "trajectory_msgs" "gz_sim_vendor" "gz_plugin_vendor")
set(ruka_BUILDTOOL_EXPORT_DEPENDS )
set(ruka_EXEC_DEPENDS "moveit_ros_move_group" "moveit_kinematics" "moveit_planners" "moveit_simple_controller_manager" "joint_state_publisher" "joint_state_publisher_gui" "tf2_ros" "xacro" "controller_manager" "joint_state_broadcaster" "controller_manager" "moveit_configs_utils" "moveit_ros_move_group" "moveit_ros_visualization" "moveit_ros_warehouse" "moveit_ros_planning_interface" "moveit_planners_ompl" "moveit_setup_assistant" "launch_ros" "launch" "robot_state_publisher" "ros2_control_demo_description" "ros2controlcli" "ros2launch" "rviz2" "rviz_common" "rviz_default_plugins" "tf2_ros" "warehouse_ros_mongo" "urdf" "xacro" "control_msgs" "controller_interface" "hardware_interface" "gz_ros2_control" "kdl_parser" "pluginlib" "rclcpp_lifecycle" "rclcpp" "rcpputils" "realtime_tools" "trajectory_msgs" "gz_sim_vendor" "gz_plugin_vendor")
set(ruka_TEST_DEPENDS "ament_cmake_gtest" "ament_cmake_pytest" "launch_testing_ament_cmake" "launch_testing_ros" "liburdfdom-tools" "xacro")
set(ruka_GROUP_DEPENDS )
set(ruka_MEMBER_OF_GROUPS )
set(ruka_DEPRECATED "")
set(ruka_EXPORT_TAGS)
list(APPEND ruka_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
