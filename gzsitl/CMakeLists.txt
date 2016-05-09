cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Generate Mavlink Messages
if(NOT EXISTS ${CMAKE_SOURCE_DIR}/build/lib/)
    execute_process(
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/../modules/mavlink
            COMMAND
            python
            -m pymavlink.tools.mavgen
            -o ${CMAKE_SOURCE_DIR}/build/lib
            --lang C
            ./message_definitions/v1.0/common.xml)
endif()

# Find Gazebo and Includes
find_package(gazebo REQUIRED)
include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/build/lib/common
    ${CMAKE_SOURCE_DIR}/build/lib)
link_directories(
    ${GAZEBO_LIBRARY_DIRS})
set(CUSTOM_COMPILE_FLAGS "-g -Wall")
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} ${CUSTOM_COMPILE_FLAGS}")

# Build Plugin
add_library(gzsitl_plugin SHARED gzsitl_plugin.cc)
target_link_libraries(gzsitl_plugin ${GAZEBO_libraries})

# Set Gazebo Environment Variables
