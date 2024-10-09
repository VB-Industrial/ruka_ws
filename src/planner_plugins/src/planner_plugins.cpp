
//#include <planner_base/regular_planner.hpp>
#include <planner_plugins/planner_plugins.hpp>
#include <cmath>

namespace planner_plugins
{
  
     PlannerPlugins::PlannerPlugins()
     {
     }

     PlannerPlugins::~PlannerPlugins()
     {
     }
  
  
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(planner_plugins::PlannerPlugins, kinematics::KinematicsBase)

