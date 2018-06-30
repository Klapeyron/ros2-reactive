cmake_minimum_required(VERSION 3.0.2)

project(ros2-reactive)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/modules)

find_package(rclcpp)
find_package(rxcpp REQUIRED)

add_library(ros2-reactive INTERFACE)
target_compile_definitions(ros2-reactive INTERFACE LIBRARY_HEADER_ONLY)
target_include_directories(ros2-reactive INTERFACE
					 ros2-reactive/
					 ${RCLCPP_INCLUDE_DIRS}
					 ${RXCPP_INCLUDE_DIRS})

include(GNUInstallDirs)
install(DIRECTORY ros2-reactive DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
