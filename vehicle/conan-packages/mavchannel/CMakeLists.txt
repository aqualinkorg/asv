cmake_minimum_required(VERSION 3.5)

set( LIB_NAME mavchannel )
project( lib${LIB_NAME} LANGUAGES CXX )

# Setup Conan
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

##############################################
# Create target
add_library( ${LIB_NAME} "" )

##############################################
# Set properties

file(GLOB_RECURSE SOURCES "src/**.cpp")
message( spurces: ${SOURCES})

# Sources
target_sources( ${LIB_NAME}
    PRIVATE
        ${SOURCES}
)

# Includes
target_include_directories( ${LIB_NAME} 
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

# Library dependencies
target_link_libraries( ${LIB_NAME} 
    PUBLIC
      CONAN_PKG::serial
      CONAN_PKG::mavlink2
      CONAN_PKG::readerwriterqueue
      CONAN_PKG::concurrentqueue
)

# Set public headers
set_target_properties( ${LIB_NAME} 
    PROPERTIES 
        PUBLIC_HEADER $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> 
)

# Compile features
target_compile_features( ${LIB_NAME} 
    PUBLIC 
        cxx_std_14 
)