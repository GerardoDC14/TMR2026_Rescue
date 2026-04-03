# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_jaguar_full_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED jaguar_full_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(jaguar_full_FOUND FALSE)
  elseif(NOT jaguar_full_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(jaguar_full_FOUND FALSE)
  endif()
  return()
endif()
set(_jaguar_full_CONFIG_INCLUDED TRUE)

# output package information
if(NOT jaguar_full_FIND_QUIETLY)
  message(STATUS "Found jaguar_full: 0.3.0 (${jaguar_full_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'jaguar_full' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${jaguar_full_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(jaguar_full_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${jaguar_full_DIR}/${_extra}")
endforeach()
