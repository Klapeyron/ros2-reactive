find_path(RCLCPP_INCLUDE_DIR NAMES rclcpp)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(rclcpp DEFAULT_MSG RCLCPP_INCLUDE_DIR)

if(RCLCPP_FOUND)
  set(RCLCPP_INCLUDE_DIRS ${RCLCPP_INCLUDE_DIR})
else()
  message(FATAL_ERROR "rclcpp package not found")
endif(RCLCPP_FOUND)

mark_as_advanced(RCLCPP_INCLUDE_DIRS)
