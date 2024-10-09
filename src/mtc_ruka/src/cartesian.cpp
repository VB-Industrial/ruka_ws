

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
  task.stages()->setName("TRY task");
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
  cartesian_planner->setStepSize(0.01);

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

//  for (int j = 0; j < 50; j++)
//   {
  
      for (int i = 0; i < 300; i++)
      {
        



        {  // perform a Cartesian motion, defined as a relative offset in joint space
            auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
            stage->setGroup(arm_group_name);
            std::map<std::string, double> offsets = {{ "base_link__link_01", 0*3.14/180}, { "link_01__link_02", -90*3.14/180}, { "link_02__link_03", 180*3.14/180 }, 
            { "link_03__link_04", -125*3.14/180 }, { "link_04__link_05", 48*3.14/180},  { "link_05__link_06", -172*3.14/180}};
            stage->setGoal(offsets);
            place->insert(std::move(stage));
            //task.add(std::move(stage));
        }
        {  // perform a Cartesian motion, defined as a relative offset in joint space
            auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
            stage->setGroup(arm_group_name);
            std::map<std::string, double> offsets = {{ "base_link__link_01", 0*3.14/180}, { "link_01__link_02", -90*3.14/180}, { "link_02__link_03", 180*3.14/180 }, 
            { "link_03__link_04", 100*3.14/180 }, { "link_04__link_05", -90*3.14/180},  { "link_05__link_06", 63*3.14/180}};
            stage->setGoal(offsets);
            place->insert(std::move(stage));
            //task.add(std::move(stage));
        }
        // {  // perform a Cartesian motion, defined as a relative offset in joint space
        //     auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
        //     stage->setGroup(arm_group_name);
        //     std::map<std::string, double> offsets = {{ "base_link__link_01", 20*3.14/180}, { "link_01__link_02", -65*3.14/180}, { "link_02__link_03", 110*3.14/180 }, 
        //     { "link_03__link_04", 1*3.14/180 }, { "link_04__link_05", -27*3.14/180},  { "link_05__link_06", 33*3.14/180}};
        //     stage->setGoal(offsets);
        //     place->insert(std::move(stage));
        //     //task.add(std::move(stage));
        // }
        // ////
        // {  // perform a Cartesian motion, defined as a relative offset in joint space
        //     auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
        //     stage->setGroup(arm_group_name);
        //     std::map<std::string, double> offsets = {{ "base_link__link_01", -103*3.14/180}, { "link_01__link_02", -158*3.14/180}, { "link_02__link_03", 239*3.14/180 }, 
        //     { "link_03__link_04", -2*3.14/180 }, { "link_04__link_05", -30*3.14/180},  { "link_05__link_06", -57*3.14/180}};
        //     stage->setGoal(offsets);
        //     place->insert(std::move(stage));
        //     //task.add(std::move(stage));
        // }


        //////
        //      {  // perform a Cartesian motion, defined as a relative offset in joint space
        //       auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
        //       stage->setGroup(arm_group_name);
        //       std::map<std::string, double> offsets = {{ "base_link__link_01", 129*3.14/180}, { "link_01__link_02", -38*3.14/180}, { "link_02__link_03", 128*3.14/180 }, 
        //       { "link_03__link_04", 2*3.14/180 }, { "link_04__link_05", -56*3.14/180},  { "link_05__link_06", -7*3.14/180}};
        //       stage->setGoal(offsets);
        //       place->insert(std::move(stage));
        //       //task.add(std::move(stage));
        //     }

        //      {  // perform a Cartesian motion, defined as a relative offset in joint space
        //       auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
        //       stage->setGroup(arm_group_name);
        //       std::map<std::string, double> offsets = {{ "base_link__link_01", 110*3.14/180}, { "link_01__link_02", -18*3.14/180}, { "link_02__link_03", 148*3.14/180 }, 
        //       { "link_03__link_04", 2*3.14/180 }, { "link_04__link_05", -65*3.14/180},  { "link_05__link_06", 7*3.14/180}};
        //       stage->setGoal(offsets);
        //       place->insert(std::move(stage));
        //       //task.add(std::move(stage));
        //     }

        //      {  // perform a Cartesian motion, defined as a relative offset in joint space
        //       auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
        //       stage->setGroup(arm_group_name);
        //       std::map<std::string, double> offsets = {{ "base_link__link_01", 84*3.14/180}, { "link_01__link_02", -33*3.14/180}, { "link_02__link_03", 130*3.14/180 }, 
        //       { "link_03__link_04", 2*3.14/180 }, { "link_04__link_05", -50*3.14/180},  { "link_05__link_06", -7*3.14/180}};
        //       stage->setGoal(offsets);
        //       place->insert(std::move(stage));
        //       //task.add(std::move(stage));
        //     }

        // {  // perform a Cartesian motion, defined as a relative offset in joint space
        //       auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
        //       stage->setGroup(arm_group_name);
        //       std::map<std::string, double> offsets = {{ "base_link__link_01", -1*3.14/180}, { "link_01__link_02", -72*3.14/180}, { "link_02__link_03", 142*3.14/180 }, 
        //       { "link_03__link_04", 2*3.14/180 }, { "link_04__link_05", -63*3.14/180},  { "link_05__link_06", 7*3.14/180}};
        //       stage->setGoal(offsets);
        //       place->insert(std::move(stage));
        //       //task.add(std::move(stage));
        //     }
      }


    // {  // perform a Cartesian motion, defined as a relative offset in joint space
    //   auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
    //   stage->setGroup(arm_group_name);
    //   std::map<std::string, double> offsets = {{ "base_link__link_01", 0*3.14/180}, { "link_01__link_02", -107*3.14/180}, { "link_02__link_03", 45*3.14/180 }, 
    //   { "link_03__link_04", 0*3.14/180 }, { "link_04__link_05", 62*3.14/180},  { "link_05__link_06", 0*3.14/180}};
    //   stage->setGoal(offsets);
    //   place->insert(std::move(stage));
    //   //task.add(std::move(stage));
    // }

    //         //   {  // perform a Cartesian motion, defined as a relative offset in joint space
    //         //   auto stage = std::make_unique<mtc::stages::MoveTo>("HELP", cartesian_planner);
    //         //   stage->setGroup(arm_group_name);
    //         //   std::map<std::string, double> offsets = {{ "base_link__link_01", 0*3.14/180}, { "link_01__link_02", -104*3.14/180}, { "link_02__link_03", 134*3.14/180 }, 
    //         //   { "link_03__link_04", 0*3.14/180 }, { "link_04__link_05", -30*3.14/180},  { "link_05__link_06", 0*3.14/180}};
    //         //   stage->setGoal(offsets);
    //         //   place->insert(std::move(stage));
    //         //   //task.add(std::move(stage));
    //         // }

    // {
    //   auto stage = std::make_unique<mtc::stages::MoveRelative>("z", cartesian_planner);
    //   stage->setGroup(arm_group_name);
    //   geometry_msgs::msg::Vector3Stamped direction;
    //   direction.header.frame_id = "world";
    //   direction.vector.z = 0.3;
     
      
    //   stage->setDirection(direction);
    //   place->insert(std::move(stage));
    //   //task.add(std::move(stage));
    // }
    // {
    // auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    // stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    // stage->setGoal("g");
    // task.add(std::move(stage));
    // }
    // {
    //   auto stage = std::make_unique<mtc::stages::MoveRelative>("zz", cartesian_planner);
    //   stage->setGroup(arm_group_name);
    //   geometry_msgs::msg::Vector3Stamped direction;
    //   direction.header.frame_id = "world";
    //   direction.vector.z = 0.1;
     
      
    //   stage->setDirection(direction);
    //   place->insert(std::move(stage));
    //   //task.add(std::move(stage));
    // }
  
        task.add(std::move(place));
}

  ///////////////////////////////////////////
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("g");
    task.add(std::move(stage));
  }
  return task;


//   {
//     auto place = std::make_unique<mtc::SerialContainer>("example case");
//     task.properties().exposeTo(place->properties(), { "group", "ik_frame" });
//     // clang-format off
//     place->properties().configureInitFrom(mtc::Stage::PARENT,
//                                           { "group", "ik_frame" });
//     // clang-format on

//     /****************************************************
//   ---- *               Generate Place Pose                *
//      ***************************************************/

//   //   {
//   //     auto stage = std::make_unique<mtc::stages::MoveRelative>("1", interpolation_planner);
//   //     stage->setGroup(arm_group_name);
//   //     geometry_msgs::msg::Vector3Stamped direction;
//   //     direction.header.frame_id = "world";
//   //     direction.vector.z = 0.09;
//   //     direction.vector.y = 0.04;
//   //     direction.vector.x = -0.05;
      
//   //     stage->setDirection(direction);
//   //     place->insert(std::move(stage));
//   //     //task.add(std::move(stage));
//   //   }
  
//   // {
//   //     auto stage = std::make_unique<mtc::stages::MoveRelative>("2", interpolation_planner);
//   //     stage->setGroup(arm_group_name);
//   //     geometry_msgs::msg::Vector3Stamped direction;
//   //     direction.header.frame_id = "world";
//   //     direction.vector.z = -0.1;
//   //     direction.vector.y = -0.2;
      
//   //     stage->setDirection(direction);
//   //     place->insert(std::move(stage));
//   //     //task.add(std::move(stage));
//   //   }


//     {
//       auto stage = std::make_unique<mtc::stages::MoveTo>("try_3", interpolation_planner);
//       stage->setGroup(arm_group_name);
//       stage->setGoal("try_3");
//       place->insert(std::move(stage));
//     }

//     //   {  // perform a Cartesian motion, defined as a relative offset in joint space
//     //   auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
//     //   stage->setGroup(arm_group_name);
//     //   std::map<std::string, double> offsets = { { "link_04__link_05", M_PI / 6. }, { "link_05__link_06", -M_PI / 6 } };
//     //   stage->setGoal(offsets);
//     //   place->insert(std::move(stage));
//     //   //task.add(std::move(stage));
//     // }

//     //  {  // perform a Cartesian motion, defined as a relative offset in joint space
//     //   auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
//     //   stage->setGroup(arm_group_name);
//     //   std::map<std::string, double> offsets = { { "link_02__link_03", 0.785 }};
//     //   stage->setGoal(offsets);
//     //   place->insert(std::move(stage));
//     //   //task.add(std::move(stage));
//     // }

//     // for 1 joint 

//   //  {  // perform a Cartesian motion, defined as a relative offset in joint space
//   //     auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
//   //     stage->setGroup(arm_group_name);
//   //     std::map<std::string, double> offsets = { { "base_link__link_01", 0.645 }, { "link_01__link_02", -2.023 }, { "link_02__link_03", 0.666 }, { "link_04__link_05", -0.2617}};
//   //     stage->setGoal(offsets);
//   //     place->insert(std::move(stage));
//   //     //task.add(std::move(stage));
//   //   }
// //for 4 joint
//   //  {  // perform a Cartesian motion, defined as a relative offset in joint space
//   //     auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
//   //     stage->setGroup(arm_group_name);
//   //     std::map<std::string, double> offsets = { { "base_link__link_01", 1.168 }, { "link_01__link_02", -2.18 }, { "link_02__link_03", 0.8373 }, { "link_03__link_04", 1.57}};
//   //     stage->setGoal(offsets);
//   //     place->insert(std::move(stage));
//   //     //task.add(std::move(stage));
//   //   }

// // //for 5 joint
// //    {  // perform a Cartesian motion, defined as a relative offset in joint space
// //       auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
// //       stage->setGroup(arm_group_name);
// //       std::map<std::string, double> offsets = { { "base_link__link_01", 1.1339 }, { "link_01__link_02", -2.006 }, { "link_02__link_03", 0.715 }, { "link_03__link_04", 1.6049}, { "link_04__link_05", 0.0}};
// //       stage->setGoal(offsets);
// //       place->insert(std::move(stage));
// //       //task.add(std::move(stage));
// //     }

     

// //   {
// //   auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
// //       "move to place",
// //       mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, interpolation_planner } });
// //   stage_move_to_place->setTimeout(3.0);
// //   stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);

// //   place->insert(std::move(stage_move_to_place));
// //   //task.add(std::move(stage_move_to_place));
// // }


// for (int i = 0; i < 21; i++)
//  {
//     // {  // perform a Cartesian motion, defined as a relative offset in joint space
//     //   auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
//     //   stage->setGroup(arm_group_name);
//     //   std::map<std::string, double> offsets = { { "link_02__link_03", 1.05 }};
//     //   stage->setGoal(offsets);
//     //   place->insert(std::move(stage));
//     //   //task.add(std::move(stage));
//     // }


//     //  {  // perform a Cartesian motion, defined as a relative offset in joint space
//     //   auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
//     //   stage->setGroup(arm_group_name);
//     //   std::map<std::string, double> offsets = { { "link_02__link_03", 0.74 }};
//     //   stage->setGoal(offsets);
//     //   place->insert(std::move(stage));
//     //   //task.add(std::move(stage));
//     // }

//     // {  // perform a Cartesian motion, defined as a relative offset in joint space
//     //   auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
//     //   stage->setGroup(arm_group_name);
//     //   std::map<std::string, double> offsets = { { "base_link__link_01", 0.0 }};
//     //   stage->setGoal(offsets);
//     //   place->insert(std::move(stage));
//     //   //task.add(std::move(stage));
//     // }


//     //  {  // perform a Cartesian motion, defined as a relative offset in joint space
//     //   auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
//     //   stage->setGroup(arm_group_name);
//     //   std::map<std::string, double> offsets = { { "base_link__link_01", 0.645 }};
//     //   stage->setGoal(offsets);
//     //   //stage->setTimeout(3.0);
//     //   place->insert(std::move(stage));
//     //   //task.add(std::move(stage));
//     // }


// // {  // perform a Cartesian motion, defined as a relative offset in joint space
// //       auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
// //       stage->setGroup(arm_group_name);
// //       std::map<std::string, double> offsets = {{ "link_04__link_05", -0.52}};
// //       stage->setGoal(offsets);
// //       place->insert(std::move(stage));
// //       //task.add(std::move(stage));
// //     }

// //     {  // perform a Cartesian motion, defined as a relative offset in joint space
// //       auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
// //       stage->setGroup(arm_group_name);
// //       std::map<std::string, double> offsets = { { "link_04__link_05", 0.0}};
// //       stage->setGoal(offsets);
// //       place->insert(std::move(stage));
// //       //task.add(std::move(stage));
// //     }

// {  // perform a Cartesian motion, defined as a relative offset in joint space
//       auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
//       stage->setGroup(arm_group_name);
//       std::map<std::string, double> offsets = {{ "base_link__link_01", 1.308 }, { "link_01__link_02", -1.727}, { "link_02__link_03", 0.785 }, { "link_03__link_04", 0.0 },
//       { "link_04__link_05", -0.61}};
//       stage->setGoal(offsets);
//       place->insert(std::move(stage));
//       //task.add(std::move(stage));
//     }

//     {  // perform a Cartesian motion, defined as a relative offset in joint space
//       auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
//       stage->setGroup(arm_group_name);
//       std::map<std::string, double> offsets = {{ "base_link__link_01", 1.74 }, { "link_01__link_02", -1.849}, { "link_02__link_03", 1.831 }, { "link_03__link_04", -0.628 },
//       { "link_04__link_05", -1.535}};
//       stage->setGoal(offsets);
//       place->insert(std::move(stage));
//       //task.add(std::move(stage));
//     }




//   //    {
// 	// 	// Connect the grasped state to the pre-place state, i.e. realize the object transport
// 	// 	auto stage = std::make_unique<mtc::stages::Connect>(
// 	// 	    "move to place", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, interpolation_planner } });
// 	// 	stage->setTimeout(3.0);
// 	// //	stage->properties().configureInitFrom(mtc::Stage::PARENT);
// 	// 	task.add(std::move(stage));
// 	// }
//     // {
//     //     auto stage = std::make_unique<mtc::stages::Connect>(
//     //         "move to place",
//     //         mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, interpolation_planner } });
//     //     stage->setTimeout(1.0);
//     //     stage->properties().configureInitFrom(mtc::Stage::PARENT);

//     //     place->insert(std::move(stage));
//     //     //task.add(std::move(stage_move_to_place));
//     // }

//   }

// //  {  // perform a Cartesian motion, defined as a relative offset in joint space
// //       auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
// //       stage->setGroup(arm_group_name);
// //       std::map<std::string, double> offsets = {{ "link_04__link_05", -0.52}};
// //       stage->setGoal(offsets);
// //       place->insert(std::move(stage));
// //       //task.add(std::move(stage));
// //     }

//     // {  // perform a Cartesian motion, defined as a relative offset in joint space
//     //   auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset", interpolation_planner);
//     //   stage->setGroup(arm_group_name);
//     //   std::map<std::string, double> offsets = { { "link_02__link_03",  M_PI / 20. }, { "link_05__link_06", -M_PI / 6 }};
//     //   stage->setGoal(offsets);
//     //   place->insert(std::move(stage));
//     //   //task.add(std::move(stage));
//     // }

//     // {  // perform a Cartesian motion, defined as a relative offset in joint space
//     //   auto stage = std::make_unique<mtc::stages::MoveTo>("joint offset min", interpolation_planner);
//     //   stage->setGroup(arm_group_name);
//     //   std::map<std::string, double> offsets = { { "link_02__link_03",  -M_PI / 20. }, { "link_05__link_06", -M_PI / 6 }};
//     //   stage->setGoal(offsets);
//     //   place->insert(std::move(stage));
//     //   //task.add(std::move(stage));
//     // }
  

//     // {
//     //   auto stage = std::make_unique<mtc::stages::MoveTo>("vertical", interpolation_planner);
//     //   stage->setGroup(arm_group_name);
//     //   stage->setGoal("vertical");
//     //   place->insert(std::move(stage));
//     // }
//     // {
// 		// // Cartesian and PTP motion to target would be in collision
// 		// auto fixed{ std::make_unique<mtc::stages::FixedState>("getting to target requires collision avoidance") };
// 		// //auto scene{ initial_scene->diff() };
// 		// //scene->getCurrentStateNonConst().setVariablePositions({ { "panda_joint1", -TAU / 8 } });
// 		// scene->processCollisionObjectMsg([]() {
// 		// 	moveit_msgs::msg::CollisionObject co;
// 		// 	//co.id = "box";
// 		// 	co.header.frame_id = "world";
// 		// 	co.operation = co.ADD;
// 		// 	co.pose = []() {
// 		// 		geometry_msgs::msg::Pose p;
// 		// 		p.position.x = 0.3;
// 		// 		p.position.y = 0.0;
// 		// 		p.position.z = 0.64 / 2;
// 		// 		p.orientation.w = 1.0;
// 		// 		return p;
// 		// 	}();
// 		// 	// co.primitives.push_back([]() {
// 		// 	// 	shape_msgs::msg::SolidPrimitive sp;
// 		// 	// 	sp.type = sp.BOX;
// 		// 	// 	sp.dimensions = { 0.2, 0.05, 0.64 };
// 		// 	// 	return sp;
// 		// 	// }());
// 		// 	return co;
// 		// }());
// 		// fixed->setState(scene);
// 		// place->add(std::move(fixed));
// 	  // }

// ////////////////////////////////////

// //  {
// // 			// Generate Place Pose
// // 			auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("coordinates");
// // 			stage->properties().configureInitFrom(mtc::Stage::PARENT, { "ik_frame" });
     
// // 			stage->properties().set("marker_ns", "place_pose");
// // 			//stage->setObject(params.object_name);

// // 			// Set target pose
// // 			geometry_msgs::msg::PoseStamped p;
// // 			p.header.frame_id = "world";
// // 			//p.pose = vectorToPose(params.place_pose);
			
// //       p.pose.position.x = 0.3;
// // 			p.pose.position.y = 0.0;
// // 			p.pose.position.z = 0.5 ;
// // 			p.pose.orientation.w = 1.0;

// // 			stage->setPose(p);
// // 		  // stage->setMonitoredStage(pick_stage_ptr);  // hook into successful pick solutions

// // 			// Compute IK
// // 		//	auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
// // 			//wrapper->setMaxIKSolutions(2);
// // 			//wrapper->setIKFrame(vectorToEigen(params.grasp_frame_transform), params.hand_frame);
// // 			//wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
// // 			//wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
// // 			place->insert(std::move(stage));
// // 		}
// //////////////////////////////////////





//     // // {
//     // //   auto stage = std::make_unique<mtc::stages::MoveRelative>("y", cartesian_planner);
//     // //   stage->setGroup(arm_group_name);
//     // //   geometry_msgs::msg::Vector3Stamped direction;
//     // //   direction.header.frame_id = "world";
//     // //   direction.vector.z = 0.025;
      
//     // //   stage->setDirection(direction);
//     // //   place->insert(std::move(stage));
//     // //   //task.add(std::move(stage));
//     // // }



    

//         task.add(std::move(place));
//   }

//   // {
// 	// 	auto stage = std::make_unique<mtc::stages::MoveRelative>("z", interpolation_planner);
// 	// 	stage->setGroup(arm_group_name);
// 	// 	geometry_msgs::msg::Vector3Stamped direction;
// 	// 	direction.header.frame_id = "world";
// 	// 	direction.vector.z = -0.05;
// 	// 	stage->setDirection(direction);
// 	// 	task.add(std::move(stage));
// 	// }
//   // {
// 	// 	auto stage = std::make_unique<mtc::stages::MoveRelative>("x", interpolation_planner);
// 	// 	stage->setGroup(arm_group_name);
// 	// 	geometry_msgs::msg::Vector3Stamped direction;
// 	// 	direction.header.frame_id = "world";
// 	// 	direction.vector.x = -0.04;
// 	// 	stage->setDirection(direction);
// 	// 	task.add(std::move(stage));
// 	// }

// // for (int i = 0; i < 3; i++)
// // {

// //  { 
// // 		auto stage = std::make_unique<mtc::stages::MoveRelative>("y", interpolation_planner);
// // 		stage->setGroup(arm_group_name);
// // 		geometry_msgs::msg::Vector3Stamped direction;
// // 		direction.header.frame_id = "world";
// // 		direction.vector.y = 0.01;
// // 		stage->setDirection(direction);
// // 		task.add(std::move(stage));
// // 	}

// // }


//   {
//     auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
//     stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//     stage->setGoal("g");
//     task.add(std::move(stage));
//   }
//   return task;
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