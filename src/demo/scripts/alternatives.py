#! /usr/bin/env python
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
from py_binding_tools import roscpp_init
import time

roscpp_init("mtc_tutorial_alternatives")

# Use the joint interpolation planner
jointPlanner = core.JointInterpolationPlanner()

# Create a task
task = core.Task()

# Start from current robot state
currentState = stages.CurrentState("current state")

# Add the current state to the task hierarchy
task.add(currentState)

# [initAndConfigAlternatives]
# The alternatives stage supports multiple execution paths
alternatives = core.Alternatives("Alternatives")

# goal 1
goalConfig1 = {
    "base_link__link_01": 1.0,
    "link_01__link_02": -1.0,
    "link_02__link_03": 0.0,
    "link_03__link_04": -2.5,
    "link_04__link_05": 1.0,
    "link_05__link_06": 1.0
}

# goal 2
goalConfig2 = {
    "base_link__link_01": -3.0,
    "link_01__link_02": -1.0,
    "link_02__link_03": 0.0,
    "link_03__link_04": -2.0,
    "link_04__link_05": 1.0,
    "link_05__link_06": 2.0
}

# First motion plan to compare
moveTo1 = stages.MoveTo("Move To Goal Configuration 1", jointPlanner)
moveTo1.group = "ruka_arm_controller"
moveTo1.setGoal(goalConfig1)
alternatives.insert(moveTo1)

# Second motion plan to compare
moveTo2 = stages.MoveTo("Move To Goal Configuration 2", jointPlanner)
moveTo2.group = "ruka_arm_controller"
moveTo2.setGoal(goalConfig2)
alternatives.insert(moveTo2)

# Add the alternatives stage to the task hierarchy
task.add(alternatives)
# [initAndConfigAlternatives]

if task.plan():
    task.publish(task.solutions[0])
time.sleep(1)
