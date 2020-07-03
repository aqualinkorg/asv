cmake_minimum_required(VERSION 3.5)
project(asv_bridge)

# Setup Conan
include(${CMAKE_CURRENT_SOURCE_DIR}/../.build/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable( ${PROJECT_NAME} 
  src/main.cpp
  src/asv_bridge_node.cpp
)

ament_target_dependencies( ${PROJECT_NAME} 
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "std_msgs"
)

target_include_directories( ${PROJECT_NAME}  
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries( ${PROJECT_NAME} 
  CONAN_PKG::spdlog
  CONAN_PKG::mavchannel
  CONAN_PKG::serial
  CONAN_PKG::boost
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()