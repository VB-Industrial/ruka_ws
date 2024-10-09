cmake_minimum_required(VERSION 3.8)
project(planner_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)



add_library(planner_plugins src/planner_plugins.cpp)
target_compile_features(planner_plugins PUBLIC c_std_99 cxx_std_17)
target_include_directories(planner_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  planner_plugins
  "moveit_core"
  "pluginlib"
)

target_compile_definitions(planner_plugins PRIVATE "PLANNER_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(kinematics plugins.xml)


install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS planner_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  planner_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)



ament_package()
