#ifndef PLANNER_BASE_REGULAR_PLANNER_HPP
#define PLANNER_BASE_REGULAR_PLANNER_HPP

namespace planner_base
{
  class RegularPlanner
  {
    public:
      virtual void initialize(double side_length) = 0;
      virtual double area() = 0;
      virtual ~RegularPlanner(){}

    protected:
      RegularPlanner(){}
  };
}  // namespace polygon_base

#endif  // PLANNER_BASE_REGULAR_PLANNER_HPP
