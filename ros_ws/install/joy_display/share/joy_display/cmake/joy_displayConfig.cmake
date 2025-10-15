# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_joy_display_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED joy_display_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(joy_display_FOUND FALSE)
  elseif(NOT joy_display_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(joy_display_FOUND FALSE)
  endif()
  return()
endif()
set(_joy_display_CONFIG_INCLUDED TRUE)

# output package information
if(NOT joy_display_FIND_QUIETLY)
  message(STATUS "Found joy_display: 0.0.0 (${joy_display_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'joy_display' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT joy_display_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(joy_display_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${joy_display_DIR}/${_extra}")
endforeach()
