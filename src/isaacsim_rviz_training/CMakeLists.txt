cmake_minimum_required(VERSION 3.5.0)
project(isaacsim_rviz_training)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# set help variables to find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
# Hardware interface related dependencies
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()



add_library(
  isaacsim_rviz_training
  SHARED
  hardware/dof6bot_hardware_interface.cpp
)


target_compile_features(isaacsim_rviz_training PUBLIC cxx_std_17)
target_include_directories(isaacsim_rviz_training PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/isaacsim_rviz_training>
)

ament_target_dependencies(
  isaacsim_rviz_training PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
  ${REF_GEN_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ISAACSIM_RVIZ_TRAINING_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface isaacsim_rviz_training.xml)


# INSTALL
install(
  DIRECTORY
    bringup
    controller
    description
    hardware
    isaacsim
  DESTINATION
    share/${PROJECT_NAME}/
)
install(
  PROGRAMS
    controller/command_slrobot.py
    isaacsim/isaac_sim_node.py
    controller/helper_functions/load_ros_parameters.py
  DESTINATION 
    lib/${PROJECT_NAME}
)


install(TARGETS isaacsim_rviz_training
  EXPORT isaacsim_rviz_training
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


## EXPORTS
ament_export_targets(isaacsim_rviz_training HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})


ament_package()
