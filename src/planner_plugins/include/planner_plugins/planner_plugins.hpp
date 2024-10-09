#ifndef PLANNER_PLUGINS__PLANNER_PLUGINS_HPP_
#define PLANNER_PLUGINS__PLANNER_PLUGINS_HPP_

#include "planner_plugins/visibility_control.h"
#include <moveit/kinematics_base/kinematics_base.h>

namespace planner_plugins
{



class PlannerPlugins : public kinematics::KinematicsBase
{
public:
  PlannerPlugins();

  virtual ~PlannerPlugins();
};

}  // namespace planner_plugins

#endif  // PLANNER_PLUGINS__PLANNER_PLUGINS_HPP_
