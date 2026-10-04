cmake_minimum_required(VERSION 3.8)
project(thesis_hardware_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(thesis_interfaces REQUIRED)
find_package(Threads REQUIRED)

get_filename_component(
  KINOVA_ROOT_DEFAULT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../TESIS_2_DEPENDENCIES/kinova-ros/kinova_driver"
  ABSOLUTE
)
if(DEFINED ENV{KINOVA_ROOT} AND NOT "$ENV{KINOVA_ROOT}" STREQUAL "")
  set(KINOVA_ROOT_DEFAULT "$ENV{KINOVA_ROOT}")
endif()
set(
  KINOVA_ROOT
  "${KINOVA_ROOT_DEFAULT}"
  CACHE PATH "Kinova kinova_driver directory"
)

set(KINOVA_INCLUDE_DIR "${KINOVA_ROOT}/include/kinova")
set(KINOVA_LIB_DIR "${KINOVA_ROOT}/lib/x86_64-linux-gnu")
set(
  KINOVA_COMMAND_LIBRARY
  "${KINOVA_LIB_DIR}/USBCommandLayerUbuntu.so"
)

if(NOT EXISTS "${KINOVA_INCLUDE_DIR}/Kinova.API.USBCommandLayerUbuntu.h")
  message(FATAL_ERROR
    "Kinova headers not found. Set -DKINOVA_ROOT=/path/to/kinova_driver")
endif()
if(NOT EXISTS "${KINOVA_COMMAND_LIBRARY}")
  message(FATAL_ERROR
    "Kinova USB library not found: ${KINOVA_COMMAND_LIBRARY}")
endif()

function(configure_kinova_target target)
  target_include_directories(${target} PRIVATE
    "${KINOVA_INCLUDE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
  target_link_directories(${target} PRIVATE "${KINOVA_LIB_DIR}")
  target_link_libraries(${target} "-Wl,-l:USBCommandLayerUbuntu.so")
  set_target_properties(${target} PROPERTIES
    BUILD_RPATH "${KINOVA_LIB_DIR}"
    INSTALL_RPATH "${KINOVA_LIB_DIR}"
  )
endfunction()

add_executable(jaco_hardware_node src/jaco_hardware_node.cpp)
ament_target_dependencies(
  jaco_hardware_node
  diagnostic_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  thesis_interfaces
)
configure_kinova_target(jaco_hardware_node)
target_link_libraries(jaco_hardware_node Threads::Threads)

install(
  TARGETS
    jaco_hardware_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bridge_logic test/test_bridge_logic.cpp)
  target_include_directories(
    test_bridge_logic PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif()

ament_package()
