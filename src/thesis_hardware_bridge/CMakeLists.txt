cmake_minimum_required(VERSION 3.8)
project(thesis_hardware_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(KINOVA_ROOT
    "/home/luis/Escritorio/TESIS_2_DEPENDENCIES/kinova-ros/kinova_driver"
    CACHE PATH "Kinova kinova_driver directory")

set(KINOVA_INCLUDE_DIR "${KINOVA_ROOT}/include/kinova")
set(KINOVA_LIB_DIR "${KINOVA_ROOT}/lib/x86_64-linux-gnu")
set(KINOVA_COMMAND_LIBRARY
    "${KINOVA_LIB_DIR}/USBCommandLayerUbuntu.so")

if(NOT EXISTS "${KINOVA_INCLUDE_DIR}/Kinova.API.USBCommandLayerUbuntu.h")
  message(FATAL_ERROR
    "Kinova headers not found. Set -DKINOVA_ROOT=/path/to/kinova_driver")
endif()

if(NOT EXISTS "${KINOVA_COMMAND_LIBRARY}")
  message(FATAL_ERROR
    "Kinova USB library not found: ${KINOVA_COMMAND_LIBRARY}")
endif()

add_executable(jaco_readonly_node
  src/jaco_readonly_node.cpp
)

target_include_directories(jaco_readonly_node PRIVATE
  "${KINOVA_INCLUDE_DIR}"
)

ament_target_dependencies(jaco_readonly_node
  rclcpp
  sensor_msgs
)

target_link_directories(jaco_readonly_node PRIVATE
  "${KINOVA_LIB_DIR}"
)

target_link_libraries(jaco_readonly_node
  "-Wl,-l:USBCommandLayerUbuntu.so"
)

set_target_properties(jaco_readonly_node PROPERTIES
  BUILD_RPATH "${KINOVA_LIB_DIR}"
  INSTALL_RPATH "${KINOVA_LIB_DIR}"
)

install(TARGETS jaco_readonly_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
