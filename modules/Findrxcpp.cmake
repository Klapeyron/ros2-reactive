find_path(RXCPP_INCLUDE_DIRS NAMES rxcpp)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(rxcpp DEFAULT_MSG RXCPP_INCLUDE_DIRS)

mark_as_advanced(RXCPP_INCLUDE_DIRS)
