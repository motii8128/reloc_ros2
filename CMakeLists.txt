cmake_minimum_required(VERSION 3.5)
project(reloc_ros2)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()
ament_auto_add_library(target_reloc_ros2 SHARED
  DIRECTORY src
)

target_include_directories(target_reloc_ros2
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
)

rclcpp_components_register_node(target_reloc_ros2
  PLUGIN "reloc_ros2::ReLoc"
  EXECUTABLE reloc
)

target_link_libraries(target_reloc_ros2 ${OpenCV_LIBRARIES})


install(DIRECTORY
  launch
DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
)