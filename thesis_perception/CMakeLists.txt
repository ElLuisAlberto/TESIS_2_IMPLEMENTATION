cmake_minimum_required(VERSION 3.8)
project(thesis_perception)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters search segmentation)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(thesis_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(pointcloud_preprocessor_node
  src/pointcloud_preprocessor_node.cpp
)
ament_target_dependencies(pointcloud_preprocessor_node
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)
target_include_directories(pointcloud_preprocessor_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pointcloud_preprocessor_node ${PCL_LIBRARIES})
target_compile_definitions(pointcloud_preprocessor_node PRIVATE ${PCL_DEFINITIONS})

add_executable(obstacle_extractor_node
  src/obstacle_extractor_node.cpp
)
ament_target_dependencies(obstacle_extractor_node
  diagnostic_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  thesis_interfaces
  visualization_msgs
)
target_include_directories(obstacle_extractor_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(obstacle_extractor_node ${PCL_LIBRARIES})
target_compile_definitions(obstacle_extractor_node PRIVATE ${PCL_DEFINITIONS})

add_executable(perception_health_node
  src/perception_health_node.cpp
)
ament_target_dependencies(perception_health_node
  diagnostic_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS
  pointcloud_preprocessor_node
  obstacle_extractor_node
  perception_health_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
    test_perception_safety_contract
    test/test_perception_safety_contract.py
  )
endif()

ament_package()
