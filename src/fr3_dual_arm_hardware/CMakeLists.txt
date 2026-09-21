cmake_minimum_required(VERSION 3.16)
project(fr3_dual_arm_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(fr3_dual_arm_hardware SHARED
  src/fr3_dual_arm_hardware_interface.cpp
)
target_compile_features(fr3_dual_arm_hardware PUBLIC cxx_std_17)
target_include_directories(fr3_dual_arm_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fr3_dual_arm_hardware
  hardware_interface
  pluginlib
  rclcpp
)

install(TARGETS fr3_dual_arm_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(hardware_interface fr3_dual_arm_hardware_plugin.xml)

ament_package()
