

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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_ruka");
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
  task.stages()->setName("flag task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "ruka_arm_controller";
  const auto& hand_group_name = "ruka_hand_controller";
  const auto& hand_frame = "ruka_hand_controller";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(.01);

  // clang-format off
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("g", interpolation_planner);
  // clang-format on
  stage_open_hand->setGroup(arm_group_name);
  stage_open_hand->setGoal("g");
  task.add(std::move(stage_open_hand));




  {
    auto place = std::make_unique<mtc::SerialContainer>("example case");
    task.properties().exposeTo(place->properties(), { "group", "ik_frame" });
    // clang-format off
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "group", "ik_frame" });
   

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("vertical", interpolation_planner);
      stage->setGroup(arm_group_name);
      stage->setGoal("vertical");
      place->insert(std::move(stage));
    }

   

  for (int i = 0; i < 3; i++)
   {
    

    {  // perform a Cartesian motion, defined as a relative offset in joint space
      auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
      stage->setGroup(arm_group_name);
      std::map<std::string, double> offsets = {{ "base_link__link_01", 0.0 }, { "link_01__link_02", -1.29}, { "link_02__link_03", 1.675}, { "link_03__link_04", 0.0 },
      { "link_04__link_05", -0.87,  { "link_05__link_06", 0.0}};
      stage->setGoal(offsets);
      place->insert(std::move(stage));
      //task.add(std::move(stage));
    }

     {  // perform a Cartesian motion, defined as a relative offset in joint space
      auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
      stage->setGroup(arm_group_name);
      std::map<std::string, double> offsets = {{ "base_link__link_01", 2.44 }, { "link_01__link_02", -2.44}, { "link_02__link_03", 3.175 }, { "link_03__link_04", -1.92 },
      { "link_04__link_05", -1.97},  { "link_05__link_06", 0.55}};
      stage->setGoal(offsets);
      place->insert(std::move(stage));
      //task.add(std::move(stage));
    }

     {  // perform a Cartesian motion, defined as a relative offset in joint space
      auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
      stage->setGroup(arm_group_name);
      std::map<std::string, double> offsets = {{ "base_link__link_01", -2.146 }, { "link_01__link_02", -3.03}, { "link_02__link_03", 3.98 }, { "link_03__link_04", 2.355 },
      { "link_04__link_05", -1.15},  { "link_05__link_06", -0.436}};
      stage->setGoal(offsets);
      place->insert(std::move(stage));
      //task.add(std::move(stage));
    }

     {  // perform a Cartesian motion, defined as a relative offset in joint space
      auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
      stage->setGroup(arm_group_name);
      std::map<std::string, double> offsets = {{ "base_link__link_01", -0.19 }, { "link_01__link_02", -2.128}, { "link_02__link_03", 3.21 }, { "link_03__link_04", 0.471 },
      { "link_04__link_05", -1.15},  { "link_05__link_06", -0.436}};
      stage->setGoal(offsets);
      place->insert(std::move(stage));
      //task.add(std::move(stage));
    }

    {  // perform a Cartesian motion, defined as a relative offset in joint space
      auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
      stage->setGroup(arm_group_name);
      std::map<std::string, double> offsets = {{ "base_link__link_01", -2.49 }, { "link_01__link_02", -2.59}, { "link_02__link_03", 4.29 }, { "link_03__link_04", 2.44 },
      { "link_04__link_05", -0.66},  { "link_05__link_06", 0.33}};
      stage->setGoal(offsets);
      place->insert(std::move(stage));
      //task.add(std::move(stage));
    }


  }

        task.add(std::move(place));
  }

  
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("g");
    task.add(std::move(stage));
  }
  return task;
}
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

//  for (int i = 0; i < 3; i++)
// {

  mtc_task_node->doTask();
// }
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}