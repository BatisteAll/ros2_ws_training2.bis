cmake_minimum_required(VERSION 3.5.0)
project(isaacsim_rviz_training)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    launch
    models
    scripts
    config
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/command_slrobot.py
    scripts/isaac_sim_node.py
    scripts/helper_functions/load_ros_parameters.py
  DESTINATION 
    lib/${PROJECT_NAME}
)

ament_package()
