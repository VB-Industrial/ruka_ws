/*********************************************************************
 * Copyright (c) 2019 Bielefeld University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Robert Haschke
   Desc:   Planning a simple sequence of Cartesian motions
*/

// #include <moveit/task_constructor/task.h>

// #include <moveit/task_constructor/stages/fixed_state.h>
// #include <moveit/task_constructor/solvers/cartesian_path.h>
// #include <moveit/task_constructor/solvers/joint_interpolation.h>
// #include <moveit/task_constructor/stages/move_to.h>
// #include <moveit/task_constructor/stages/move_relative.h>
// #include <moveit/task_constructor/stages/connect.h>

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/planning_scene/planning_scene.h>

// using namespace moveit::task_constructor;

// Task createTask(const rclcpp::Node::SharedPtr& node) {
// 	Task t;
// 	t.stages()->setName("Cartesian Path");

// 	//const std::string group = "ruka_arm_controller";


//   const auto& group = "ruka_arm_controller";
//  // const auto& hand_group_name = "ruka_arm_controller";
//   const auto& hand_frame = "ruka_arm_controller";

//   // Set task properties
//   t.setProperty("group", group);
//  // task.setProperty("eef", hand_group_name);
//   t.setProperty("ik_frame", group);


// 	// const std::string eef = "link_05__link_06";

// 	// create Cartesian interpolation "planner" to be used in various stages
// 	auto cartesian_interpolation = std::make_shared<solvers::CartesianPath>();
// 	// create a joint-space interpolation "planner" to be used in various stages
// 	auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

// 	// start from a fixed robot state
// 	t.loadRobotModel(node);
// 	auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
// 	{
// 		auto& state = scene->getCurrentStateNonConst();
// 		state.setToDefaultValues(state.getJointModelGroup(group), "g");

// 		auto fixed = std::make_unique<stages::FixedState>("initial state");
// 		fixed->setState(scene);
// 		t.add(std::move(fixed));
// 	}

// 	// {
// 	// 	auto stage = std::make_unique<stages::MoveRelative>("zzzzzzzz", cartesian_interpolation);
// 	// 	stage->setGroup(group);
// 	// 	geometry_msgs::msg::Vector3Stamped direction;
// 	// 	direction.header.frame_id = "world";
// 	// 	direction.vector.z = 0.2;
// 	// 	stage->setDirection(direction);
// 	// 	t.add(std::move(stage));
// 	// }

//   {auto g =
//         std::make_unique<mtc::stages::MoveTo>("g", cartesian_interpolation);
//     g->setGroup(group);
//     g->setGoal("g");
//     task.add(std::move(g));}
// 	// {
// 	// 	auto stage = std::make_unique<stages::MoveRelative>("y", cartesian_interpolation);
// 	// 	stage->setGroup(group);
// 	// 	geometry_msgs::msg::Vector3Stamped direction;
// 	// 	direction.header.frame_id = "world";
// 	// 	direction.vector.y = 0.0;
// 	// 	stage->setDirection(direction);
// 	// 	t.add(std::move(stage));
// 	// }

// 	// {  // rotate about TCP
// 	// 	auto stage = std::make_unique<stages::MoveRelative>("rz +45°", cartesian_interpolation);
// 	// 	stage->setGroup(group);
// 	// 	geometry_msgs::msg::TwistStamped twist;
// 	// 	twist.header.frame_id = "world";
// 	// 	twist.twist.angular.z = M_PI / 4.;
// 	// 	stage->setDirection(twist);
// 	// 	t.add(std::move(stage));
// 	// }

// 	// {  // perform a Cartesian motion, defined as a relative offset in joint space
// 	// 	auto stage = std::make_unique<stages::MoveRelative>("joint offset", cartesian_interpolation);
// 	// 	stage->setGroup(group);
// 	// 	std::map<std::string, double> offsets = { { "base_link__link_01", M_PI / 6. }, { "link_02__link_03", -M_PI / 6 } };
// 	// 	stage->setDirection(offsets);
// 	// 	t.add(std::move(stage));
// 	// }

// 	{  // move from reached state back to the original state, using joint interpolation
// 		stages::Connect::GroupPlannerVector planners = { { group, joint_interpolation } };
// 		auto connect = std::make_unique<stages::Connect>("connect", planners);
// 		t.add(std::move(connect));
// 	}

// 	{  // final state is original state again
// 		auto fixed = std::make_unique<stages::FixedState>("final state");
// 		fixed->setState(scene);
// 		t.add(std::move(fixed));
// 	}

// 	return t;
// }

// int main(int argc, char** argv) {
// 	rclcpp::init(argc, argv);
// 	auto node = rclcpp::Node::make_shared("mtc_tutorial");
// 	std::thread spinning_thread([node] { rclcpp::spin(node); });

// 	auto task = createTask(node);
// 	try {
// 		if (task.plan())
// 			task.introspection().publishSolution(*task.solutions().front());
// 	} catch (const InitStageException& ex) {
// 		std::cerr << "planning failed with exception" << std::endl << ex << task;
// 	}

// 	// keep alive for interactive inspection in rviz
// 	spinning_thread.join();
// 	return 0;
// }



// #include <rclcpp/rclcpp.hpp>
// #include <moveit/planning_scene/planning_scene.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/task_constructor/task.h>
// #include <moveit/task_constructor/solvers.h>
// #include <moveit/task_constructor/stages.h>
// #if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #else
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #endif
// #if __has_include(<tf2_eigen/tf2_eigen.hpp>)
// #include <tf2_eigen/tf2_eigen.hpp>
// #else
// #include <tf2_eigen/tf2_eigen.h>
// #endif

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
// namespace mtc = moveit::task_constructor;

// class MTCTaskNode
// {
// public:
//   MTCTaskNode(const rclcpp::NodeOptions& options);

//   rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

//   void doTask();

//   void setupPlanningScene();

// private:
//   // Compose an MTC task from a series of stages.
//   mtc::Task createTask();
//   mtc::Task task_;
//   rclcpp::Node::SharedPtr node_;
// };

// rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
// {
//   return node_->get_node_base_interface();
// }

// MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
//   : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
// {
// }

// void MTCTaskNode::setupPlanningScene()
// {
//   moveit_msgs::msg::CollisionObject object;
//   object.id = "object";
//   object.header.frame_id = "world";
//   object.primitives.resize(1);
//   object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
//   object.primitives[0].dimensions = { 0.1, 0.02 };

//   geometry_msgs::msg::Pose pose;
//   pose.position.x = 0.5;
//   pose.position.y = -0.25;
//   pose.orientation.w = 1.0;
//   object.pose = pose;

//   moveit::planning_interface::PlanningSceneInterface psi;
//   psi.applyCollisionObject(object);
// }

// void MTCTaskNode::doTask()
// {
//   task_ = createTask();

//   try
//   {
//     task_.init();
//   }
//   catch (mtc::InitStageException& e)
//   {
//     RCLCPP_ERROR_STREAM(LOGGER, e);
//     return;
//   }

//   if (!task_.plan(5))
//   {
//     RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
//     return;
//   }
//   task_.introspection().publishSolution(*task_.solutions().front());

//   auto result = task_.execute(*task_.solutions().front());
//   if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
//   {
//     RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
//     return;
//   }

//   return;
// }

// mtc::Task MTCTaskNode::createTask()
// {
//   mtc::Task task;
//   task.stages()->setName("TRY task");
//   task.loadRobotModel(node_);

//   const auto& arm_group_name = "ruka_arm_controller";
//   const auto& hand_group_name = "ruka_arm_controller";
//   const auto& hand_frame = "ruka_arm_controller";

//   // Set task properties
//   task.setProperty("group", arm_group_name);
//   task.setProperty("eef", hand_group_name);
//   task.setProperty("ik_frame", arm_group_name);

// // Disable warnings for this line, as it's a variable that's set but not used in this example
// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
//   mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
// #pragma GCC diagnostic pop

//   auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
//   current_state_ptr = stage_state_current.get();
//   task.add(std::move(stage_state_current));

//   auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
//   auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

//  	//auto joint_interpolation = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

//   auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
//   cartesian_planner->setMaxVelocityScalingFactor(1.0);
//   cartesian_planner->setMaxAccelerationScalingFactor(1.0);
//   cartesian_planner->setStepSize(.01);

//   auto g =
//       std::make_unique<mtc::stages::MoveTo>("g", interpolation_planner);
//   g->setGroup(arm_group_name);
//   g->setGoal("g");
//   task.add(std::move(g));
// ////////////////////////////////
  
//   auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
//     "move to pick",
//     mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
//   stage_move_to_pick->setTimeout(5.0);
//   stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
//   task.add(std::move(stage_move_to_pick));

//   mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator

// //   {
// //   auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
// //   task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
// //   grasp->properties().configureInitFrom(mtc::Stage::PARENT,
// //                                         { "eef", "group", "ik_frame" });

// //   {
// //   auto stage =
// //       std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
// //   stage->properties().set("marker_ns", "approach_object");
// //   stage->properties().set("link", hand_frame);
// //   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
// //   stage->setMinMaxDistance(0.1, 0.15);

// //   // Set hand forward direction
// //   geometry_msgs::msg::Vector3Stamped vec;
// //   vec.header.frame_id = hand_frame;
// //   vec.vector.z = 1.0;
// //   stage->setDirection(vec);
// //   grasp->insert(std::move(stage));
// //   }
  
// //   {
// //   // Sample grasp pose
// //   auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
// //   stage->properties().configureInitFrom(mtc::Stage::PARENT);
// //   stage->properties().set("marker_ns", "grasp_pose");
// //   stage->setPreGraspPose("g");
// //   stage->setObject("object");
// //   stage->setAngleDelta(M_PI / 12);
// //   stage->setMonitoredStage(current_state_ptr);  // Hook into current state

// //   Eigen::Isometry3d grasp_frame_transform;
// //   Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
// //                         Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
// //                         Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
// //   grasp_frame_transform.linear() = q.matrix();
// //   grasp_frame_transform.translation().z() = 0.1;
 
// //     // Compute IK
// //   auto wrapper =
// //       std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
// //   wrapper->setMaxIKSolutions(8);
// //   wrapper->setMinSolutionDistance(1.0);
// //   wrapper->setIKFrame(grasp_frame_transform, hand_frame);
// //   wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
// //   wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
// //   grasp->insert(std::move(wrapper));
// // }

// // {
// //   auto stage =
// //       std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
// //   stage->allowCollisions("object",
// //                         task.getRobotModel()
// //                             ->getJointModelGroup(hand_group_name)
// //                             ->getLinkModelNamesWithCollisionGeometry(),
// //                         true);
// //   grasp->insert(std::move(stage));
// // }

// // {
// //   auto stage = std::make_unique<mtc::stages::MoveTo>("vertical", interpolation_planner);
// //   stage->setGroup(arm_group_name);
// //   stage->setGoal("vertical");
// //   grasp->insert(std::move(stage));
// // }

// // {
// //   auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
// //   stage->attachObject("object", hand_frame);
// //   attach_object_stage = stage.get();
// //   grasp->insert(std::move(stage));
// // }

// // {
// //   auto stage =
// //       std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
// //   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
// //   stage->setMinMaxDistance(0.1, 0.3);
// //   stage->setIKFrame(hand_frame);
// //   stage->properties().set("marker_ns", "lift_object");

// //   // Set upward direction
// //   geometry_msgs::msg::Vector3Stamped vec;
// //   vec.header.frame_id = "world";
// //   vec.vector.z = 1.0;
// //   stage->setDirection(vec);
// //   grasp->insert(std::move(stage));
// // }

// //   task.add(std::move(grasp));
// // }
//   return task;
// }

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);

//   rclcpp::NodeOptions options;
//   options.automatically_declare_parameters_from_overrides(true);

//   auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
//   rclcpp::executors::MultiThreadedExecutor executor;

//   auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
//     executor.add_node(mtc_task_node->getNodeBaseInterface());
//     executor.spin();
//     executor.remove_node(mtc_task_node->getNodeBaseInterface());
//   });

//   mtc_task_node->setupPlanningScene();
//   mtc_task_node->doTask();

//   spin_thread->join();
//   rclcpp::shutdown();
//   return 0;
// }



#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_task_constructor_demo");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = -0.25;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5 /* max_solutions */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("TRY task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "ruka_arm_controller";
  //const auto& hand_group_name = "hand";
  const auto& hand_frame = "ruka_arm_controller";

  // Set task properties
  task.setProperty("group", arm_group_name);
 // task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // clang-format off
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("g", interpolation_planner);
  // clang-format on
  stage_open_hand->setGroup(arm_group_name);
  stage_open_hand->setGoal("g");
  task.add(std::move(stage_open_hand));

  // clang-format off
  // auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
  //     "move to pick",
  //     mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  // // clang-format on
  // stage_move_to_pick->setTimeout(5.0);
  // stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  // task.add(std::move(stage_move_to_pick));

  // // clang-format off
  // mtc::Stage* attach_object_stage =
  //     nullptr;  // Forward attach_object_stage to place pose generator
  // // clang-format on

  // // This is an example of SerialContainer usage. It's not strictly needed here.
  // // In fact, `task` itself is a SerialContainer by default.
  // {
  //   auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
  //   task.properties().exposeTo(grasp->properties(), { "group", "ik_frame" });
  //   // clang-format off
  //   grasp->properties().configureInitFrom(mtc::Stage::PARENT,
  //                                         { "group", "ik_frame" });
  //   // clang-format on

  //   {
  //     // clang-format off
  //     auto stage =
  //         std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
  //     // clang-format on
  //     stage->properties().set("marker_ns", "approach_object");
  //     stage->properties().set("link", hand_frame);
  //     stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //     stage->setMinMaxDistance(0.1, 0.15);

  //     // Set hand forward direction
  //     geometry_msgs::msg::Vector3Stamped vec;
  //     vec.header.frame_id = hand_frame;
  //     vec.vector.z = 1.0;
  //     stage->setDirection(vec);
  //     grasp->insert(std::move(stage));
  //   }

  //   /****************************************************
  // ---- *               Generate Grasp Pose                *
  //    ***************************************************/
  //   // {
  //   //   // Sample grasp pose
  //   //   auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
  //   //   stage->properties().configureInitFrom(mtc::Stage::PARENT);
  //   //   stage->properties().set("marker_ns", "grasp_pose");
  //   //   stage->setPreGraspPose("g");
  //   //   stage->setObject("object");
  //   //   stage->setAngleDelta(M_PI / 12);
  //   //   stage->setMonitoredStage(current_state_ptr);  // Hook into current state

  //   //   // This is the transform from the object frame to the end-effector frame
  //   //   Eigen::Isometry3d grasp_frame_transform;
  //   //   Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
  //   //                          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
  //   //                          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  //   //   grasp_frame_transform.linear() = q.matrix();
  //   //   grasp_frame_transform.translation().z() = 0.1;

  //   //   // Compute IK
  //   //   // clang-format off
  //   //   auto wrapper =
  //   //       std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
  //   //   // clang-format on
  //   //   wrapper->setMaxIKSolutions(8);
  //   //   wrapper->setMinSolutionDistance(1.0);
  //   //   wrapper->setIKFrame(grasp_frame_transform, hand_frame);
  //   //   wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //   //   wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  //   //   grasp->insert(std::move(wrapper));
  //   // }

  //   // {
  //   //   // clang-format off
  //   //   auto stage =
  //   //       std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
  //   //   stage->allowCollisions("object",
  //   //                          task.getRobotModel()
  //   //                              ->getJointModelGroup(arm_group_name)
  //   //                              ->getLinkModelNamesWithCollisionGeometry(),
  //   //                          true);
  //   //   // clang-format on
  //   //   grasp->insert(std::move(stage));
  //   // }

  //   {
  //     auto stage = std::make_unique<mtc::stages::MoveTo>("vertical", interpolation_planner);
  //     stage->setGroup(arm_group_name);
  //     stage->setGoal("vertical");
  //     grasp->insert(std::move(stage));
  //   }

  //   // {
  //   //   auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  //   //   stage->attachObject("object", hand_frame);
  //   //   attach_object_stage = stage.get();
  //   //   grasp->insert(std::move(stage));
  //   // }

  //   // {
  //   //   // clang-format off
  //   //   auto stage =
  //   //       std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
  //   //   // clang-format on
  //   //   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //   //   stage->setMinMaxDistance(0.1, 0.3);
  //   //   stage->setIKFrame(hand_frame);
  //   //   stage->properties().set("marker_ns", "lift_object");

  //   //   // Set upward direction
  //   //   geometry_msgs::msg::Vector3Stamped vec;
  //   //   vec.header.frame_id = "world";
  //   //   vec.vector.z = 1.0;
  //   //   stage->setDirection(vec);
  //   //   grasp->insert(std::move(stage));
  //   // }
  //   task.add(std::move(grasp));
  // }

  // // {
  // //   // clang-format off
  // //   auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
  // //       "move to place",
  // //       mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
  // //                                                 { arm_group_name, interpolation_planner } });
  // //   // clang-format on
  // //   stage_move_to_place->setTimeout(5.0);
  // //   stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  // //   task.add(std::move(stage_move_to_place));
  // // }

  {
    auto place = std::make_unique<mtc::SerialContainer>("example case");
    task.properties().exposeTo(place->properties(), { "group", "ik_frame" });
    // clang-format off
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "group", "ik_frame" });
    // clang-format on

    /****************************************************
  ---- *               Generate Place Pose                *
     ***************************************************/

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("upper_1", interpolation_planner);
      stage->setGroup(arm_group_name);
      stage->setGoal("upper_1");
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("upper_2", cartesian_planner);
      stage->setGroup(arm_group_name);
      stage->setGoal("upper_2");
      place->insert(std::move(stage));
    }


    // {
    //   // clang-format off
    //   auto stage =
    //       std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
    //   stage->allowCollisions("object",
    //                          task.getRobotModel()
    //                              ->getJointModelGroup(hand_group_name)
    //                              ->getLinkModelNamesWithCollisionGeometry(),
    //                          false);
    //   // clang-format on
    //   place->insert(std::move(stage));
    // }

    // {
    //   auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    //   stage->detachObject("object", hand_frame);
    //   place->insert(std::move(stage));
    // }

    // {
    //   auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    //   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    //   stage->setMinMaxDistance(0.1, 0.3);
    //   stage->setIKFrame(hand_frame);
    //   stage->properties().set("marker_ns", "retreat");

    //   // Set retreat direction
    //   geometry_msgs::msg::Vector3Stamped vec;
    //   vec.header.frame_id = "world";
    //   vec.vector.x = -0.5;
    //   stage->setDirection(vec);
    //   place->insert(std::move(stage));
    // }
    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("vertical");
    task.add(std::move(stage));
  }
  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}