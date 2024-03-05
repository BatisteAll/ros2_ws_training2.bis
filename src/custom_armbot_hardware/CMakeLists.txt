cmake_minimum_required(VERSION 3.8)
project(custom_armbot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/dof6bot_hardware_interface.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)


target_compile_definitions(${PROJECT_NAME} PRIVATE "JOINT_STATE_BROADCASTER_BUILDING_DLL")
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(isaacsim_rviz_training dof6bot_hardware_interface.xml)


install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION
    lib
)

install(
  DIRECTORY
    include/
  DESTINATION
    include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
