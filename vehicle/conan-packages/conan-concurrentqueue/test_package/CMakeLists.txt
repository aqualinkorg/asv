project(PackageTest CXX)
cmake_minimum_required(VERSION 3.1.2)

set(CMAKE_CXX_STANDARD 11)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

add_executable(example example.cxx)
target_link_libraries(example CONAN_PKG::concurrentqueue)