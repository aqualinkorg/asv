cmake_minimum_required(VERSION 2.8.3)
project(serial)

##############################################
# Create target

add_library( ${PROJECT_NAME} "" )

##############################################

# Sources
target_sources( ${PROJECT_NAME}
    PRIVATE
        src/serial.cc
        src/impl/unix.cc
        src/impl/list_ports/list_ports_linux.cc
)

# Includes
target_include_directories( ${PROJECT_NAME} 
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

# Library dependencies
target_link_libraries( ${PROJECT_NAME} 
    PUBLIC
        rt
        pthread
)

# Set public headers
set_target_properties( ${PROJECT_NAME} 
    PROPERTIES 
        PUBLIC_HEADER $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/> 
)

## Install executable
install( TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    PUBLIC_HEADER DESTINATION include
)
