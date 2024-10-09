#include <cstdio>

#include <pluginlib/class_loader.hpp>
#include <planner_base/regular_planner.hpp>

int main(int argc, char** argv)
{
  // To avoid unused parameter warnings
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<planner_base::RegularPlanner> poly_loader("planner_base", "planner_base::RegularPlanner");

  try
  {
    std::shared_ptr<planner_base::RegularPlanner> triangle = poly_loader.createSharedInstance("planner_plugins::Triangle");
    triangle->initialize(10.0);

    std::shared_ptr<planner_base::RegularPlanner> square = poly_loader.createSharedInstance("planner_plugins::Square");
    square->initialize(10.0);

    printf("Triangle area: %.2f\n", triangle->area());
    printf("Square area: %.2f\n", square->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}
