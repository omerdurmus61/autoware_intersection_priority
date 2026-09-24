cmake_minimum_required(VERSION 3.14)
project(autoware_intersection_priority)

find_package(autoware_cmake REQUIRED)
# Discovers and links package.xml dependencies, including map conversion and tier4_v2x_msgs.
autoware_package()

ament_auto_add_executable(autoware_intersection_priority_node
  src/autoware_intersection_priority_node.cpp
)
target_compile_features(autoware_intersection_priority_node PUBLIC cxx_std_17)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE
  launch
  config
)
