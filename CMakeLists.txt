cmake_minimum_required(VERSION 3.16)
project(ros_diff_bot LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  socketcan_interface
  std_msgs
  geometry_msgs
  diagnostic_msgs
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  ros_diff_bot
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(ros_diff_bot PUBLIC cxx_std_17)
target_include_directories(ros_diff_bot PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/ros_diff_bot>
)
target_link_libraries(ros_diff_bot PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros_diff_bot.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/ros_diff_bot
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz
  DESTINATION share/ros_diff_bot
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros_diff_bot
)
install(TARGETS ros_diff_bot
  EXPORT export_ros_diff_bot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(diffbot_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_diffbot_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_diffbot_launch test/test_diffbot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_ros_diff_bot HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
