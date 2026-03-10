cmake_minimum_required(VERSION 3.12)
project(mpc-rbt-solution)

set(CMAKE_CXX_STANDARD 20)

# Enforce stricter compilation rules
add_compile_options(-Wall -Wextra -Wpedantic)

# Our binary executable names
set(SENDER_EXECUTABLE_NAME sender_node)
set(RECEIVER_EXECUTABLE_NAME receiver_node)

# Make sure all required dependencies are locatable
find_package(nlohmann_json REQUIRED)
#NOTE: We will use some functions from ROS 2 but not yet its
#      full capabilities.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Define include directories for the compiler
include_directories(PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Build and install libraries
#NOTE: Our core library will "simulate" what ROS 2 could handle
#      for us instead.
add_library(${PROJECT_NAME}-core SHARED
        src/util/Socket.cpp
        src/util/Utils.cpp
)
target_link_libraries(${PROJECT_NAME}-core
        nlohmann_json::nlohmann_json
)
ament_target_dependencies(${PROJECT_NAME}-core rclcpp)

#NOTE: Our main library will define our "nodes" similarly to how
#      we would do it if we were actually using ROS 2.
add_library(${PROJECT_NAME} SHARED
        src/Receiver.cpp
        src/Sender.cpp
)
target_link_libraries(${PROJECT_NAME}
        ${PROJECT_NAME}-core
)

# Build executables
add_executable(${SENDER_EXECUTABLE_NAME} src/sender_node.cpp)
target_link_libraries(${SENDER_EXECUTABLE_NAME} ${PROJECT_NAME})
add_executable(${RECEIVER_EXECUTABLE_NAME} src/receiver_node.cpp)
target_link_libraries(${RECEIVER_EXECUTABLE_NAME} ${PROJECT_NAME})

# Set up testing
if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  enable_testing()
  add_subdirectory(tests)
endif ()
