cmake_minimum_required(VERSION 3.8)
project(thesis_jaco_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Ceres REQUIRED CONFIG)
find_package(assimp REQUIRED CONFIG)
find_package(ament_index_cpp REQUIRED)
find_package(urdf REQUIRED)

add_library(self_filter_geometry src/self_filter_geometry.cpp)
target_include_directories(self_filter_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(self_filter_geometry PUBLIC cxx_std_17)
add_library(self_mesh_geometry src/self_mesh_geometry.cpp)
target_include_directories(self_mesh_geometry PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(self_mesh_geometry PUBLIC cxx_std_17)
target_link_libraries(self_mesh_geometry assimp::assimp)
ament_target_dependencies(self_mesh_geometry ament_index_cpp urdf)

add_library(shadow_filter_logic src/shadow_filter_logic.cpp)
target_include_directories(shadow_filter_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(shadow_filter_logic self_filter_geometry)
target_compile_features(shadow_filter_logic PUBLIC cxx_std_17)

add_library(observation_probe_logic src/observation_probe_logic.cpp)
target_include_directories(observation_probe_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(observation_probe_logic sensor_msgs)
target_compile_features(observation_probe_logic PUBLIC cxx_std_17)

add_library(calibration_candidate_logic src/calibration_candidate_logic.cpp)
target_include_directories(calibration_candidate_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(calibration_candidate_logic PUBLIC cxx_std_17)

add_library(temporary_alignment src/temporary_alignment.cpp)
target_include_directories(temporary_alignment PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(temporary_alignment self_filter_geometry Ceres::ceres)
target_compile_features(temporary_alignment PUBLIC cxx_std_17)
add_library(multipose_association src/multipose_association.cpp)
target_include_directories(multipose_association PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(multipose_association self_filter_geometry)
add_library(multipose_alignment src/multipose_alignment.cpp)
target_include_directories(multipose_alignment PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(multipose_alignment multipose_association Ceres::ceres)

add_executable(jaco_self_filter_node src/jaco_self_filter_node.cpp)
target_link_libraries(jaco_self_filter_node shadow_filter_logic)
ament_target_dependencies(jaco_self_filter_node
  diagnostic_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2_ros)

add_executable(d435i_observation_probe src/d435i_observation_probe.cpp)
target_link_libraries(d435i_observation_probe observation_probe_logic)
ament_target_dependencies(d435i_observation_probe diagnostic_msgs rclcpp sensor_msgs)

add_executable(d435i_calibration_candidate_node src/d435i_calibration_candidate_node.cpp)
ament_target_dependencies(d435i_calibration_candidate_node
  diagnostic_msgs
  geometry_msgs
  rclcpp
  sensor_msgs)

add_executable(jaco_self_residual_probe src/jaco_self_residual_probe.cpp)
target_link_libraries(jaco_self_residual_probe self_filter_geometry)
ament_target_dependencies(jaco_self_residual_probe
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2_ros)
add_executable(jaco_mesh_shadow_node src/jaco_mesh_shadow_node.cpp)
target_link_libraries(jaco_mesh_shadow_node self_mesh_geometry)
ament_target_dependencies(jaco_mesh_shadow_node diagnostic_msgs geometry_msgs rclcpp sensor_msgs tf2_ros)

install(TARGETS
  self_filter_geometry self_mesh_geometry shadow_filter_logic observation_probe_logic calibration_candidate_logic temporary_alignment multipose_association multipose_alignment
  jaco_self_filter_node d435i_observation_probe d435i_calibration_candidate_node jaco_self_residual_probe jaco_mesh_shadow_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_self_filter_geometry
    test/test_self_filter_geometry.cpp)
  target_link_libraries(test_self_filter_geometry self_filter_geometry)
  ament_add_gtest(test_self_mesh_geometry test/test_self_mesh_geometry.cpp)
  target_link_libraries(test_self_mesh_geometry self_mesh_geometry)
  target_compile_definitions(test_self_mesh_geometry PRIVATE THESIS_DESCRIPTION_XACRO="${CMAKE_CURRENT_SOURCE_DIR}/../thesis_description/urdf/j2n6s300_standalone.xacro")
  ament_add_gtest(test_mesh_shadow_logic test/test_mesh_shadow_logic.cpp)
  target_link_libraries(test_mesh_shadow_logic self_mesh_geometry)
  ament_add_gtest(test_shadow_filter_logic test/test_shadow_filter_logic.cpp)
  target_link_libraries(test_shadow_filter_logic shadow_filter_logic)
  ament_add_gtest(test_observation_probe_logic test/test_observation_probe_logic.cpp)
  target_link_libraries(test_observation_probe_logic observation_probe_logic)
  ament_add_gtest(test_calibration_candidate_logic test/test_calibration_candidate_logic.cpp)
  target_link_libraries(test_calibration_candidate_logic calibration_candidate_logic)
  ament_add_gtest(test_temporary_alignment test/test_temporary_alignment.cpp)
  target_link_libraries(test_temporary_alignment temporary_alignment)
  ament_add_gtest(test_multipose_alignment test/test_multipose_alignment.cpp)
  target_link_libraries(test_multipose_alignment multipose_alignment)
  ament_add_gtest(test_self_residual_metrics test/test_self_residual_metrics.cpp)
  target_link_libraries(test_self_residual_metrics self_filter_geometry)
  ament_add_pytest_test(test_calibration_contract test/test_calibration_contract.py)
  ament_add_pytest_test(test_shadow_session_contract test/test_shadow_session_contract.py)
  ament_add_pytest_test(test_capsule_profile_contract test/test_capsule_profile_contract.py)
  ament_add_pytest_test(test_package_boundary test/test_package_boundary.py)
endif()

ament_export_include_directories(include)
ament_export_libraries(self_filter_geometry shadow_filter_logic calibration_candidate_logic)
ament_package()
