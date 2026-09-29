cmake_minimum_required(VERSION 3.14)
project(autoware_intersection_priority)

find_package(autoware_cmake REQUIRED)
# Discovers and links package.xml dependencies, including map conversion and tier4_v2x_msgs.
autoware_package()

ament_auto_add_executable(autoware_intersection_priority_node
  src/autoware_intersection_priority_node.cpp
)
target_compile_features(autoware_intersection_priority_node PUBLIC cxx_std_17)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  add_executable(intersection_test_map test/make_test_map.cpp)
  target_compile_features(intersection_test_map PRIVATE cxx_std_17)
  ament_target_dependencies(intersection_test_map
    autoware_map_msgs autoware_lanelet2_extension lanelet2_core
  )
  ament_add_pytest_test(multi_intersection test/test_multi_intersection.py
    TIMEOUT 90
    ENV
      "ROS_LOCALHOST_ONLY=1"
      "VTL_NODE=$<TARGET_FILE:autoware_intersection_priority_node>"
      "VTL_MAP_FIXTURE=$<TARGET_FILE:intersection_test_map>"
      "VTL_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/config/autoware_intersection_priority.param.yaml"
  )
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE
  launch
  config
)
