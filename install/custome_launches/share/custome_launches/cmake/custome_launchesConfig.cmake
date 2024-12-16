# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_custome_launches_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED custome_launches_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(custome_launches_FOUND FALSE)
  elseif(NOT custome_launches_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(custome_launches_FOUND FALSE)
  endif()
  return()
endif()
set(_custome_launches_CONFIG_INCLUDED TRUE)

# output package information
if(NOT custome_launches_FIND_QUIETLY)
  message(STATUS "Found custome_launches: 0.0.0 (${custome_launches_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'custome_launches' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${custome_launches_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(custome_launches_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${custome_launches_DIR}/${_extra}")
endforeach()
