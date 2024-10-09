# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ruka_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ruka_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ruka_FOUND FALSE)
  elseif(NOT ruka_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ruka_FOUND FALSE)
  endif()
  return()
endif()
set(_ruka_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ruka_FIND_QUIETLY)
  message(STATUS "Found ruka: 0.3.0 (${ruka_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ruka' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ruka_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ruka_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${ruka_DIR}/${_extra}")
endforeach()
