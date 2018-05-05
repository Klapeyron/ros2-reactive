cmake_minimum_required(VERSION 3.0.2)

project(ros2-reactive)

# find_package(rclcpp REQUIRED)

add_library(ros2-reactive INTERFACE)
target_include_directories(ros2-reactive INTERFACE ros2-reactive/)
target_compile_definitions(ros2-reactive INTERFACE LIBRARY_HEADER_ONLY)

install(DIRECTORY ros2-reactive DESTINATION ${CMAKE_INSTALL_PREFIX}/include/)
