find_path(RXCPP_INCLUDE_DIR NAMES rxcpp)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(rxcpp DEFAULT_MSG RXCPP_INCLUDE_DIR)

if(RXCPP_FOUND)
  set(RXCPP_INCLUDE_DIRS ${RXCPP_INCLUDE_DIR})
else()
  message(FATAL_ERROR "rxcpp package not found")
endif(RXCPP_FOUND)

mark_as_advanced(RXCPP_INCLUDE_DIRS)
