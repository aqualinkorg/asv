cmake_minimum_required(VERSION 3.5)
project(recorder)

# Setup Conan
include(${CMAKE_CURRENT_SOURCE_DIR}/../.build/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # TODO: ZED SDK has some annoying issues with reordering, unused params, ignored qualifiers, and deprecated methods
  # Nothing to be done for now, except ignore them
  add_compile_options(-Wall -Wextra -Wno-reorder -Wno-unused-parameter -Wno-ignored-qualifiers -Wno-deprecated-declarations -Wno-deprecated-copy)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ZED SDK Setup
find_package(ZED 3 REQUIRED)
find_package(CUDA REQUIRED)

add_executable( ${PROJECT_NAME} 
  src/main.cpp
  src/recorder_node.cpp
  src/periodic_scheduler.cpp
)

ament_target_dependencies( ${PROJECT_NAME} 
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "std_msgs"
  "ament_index_cpp"
)

target_include_directories( ${PROJECT_NAME}  
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CUDA_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
)

link_directories( ${ZED_LIBRARY_DIR} )
link_directories( ${CUDA_LIBRARY_DIRS} )

SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
target_link_libraries( ${PROJECT_NAME} 
  CONAN_PKG::spdlog
  CONAN_PKG::boost
  CONAN_PKG::libjpeg-turbo
  ${ZED_LIBS}
)

target_compile_options( ${PROJECT_NAME}
PUBLIC
  "-O3"
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()