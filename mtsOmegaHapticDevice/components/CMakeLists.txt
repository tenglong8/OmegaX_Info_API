#
# (C) Copyright 2016-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (mtsOmegaHapticDevice VERSION 0.1.0)

# So we can use Findgclib.cmake
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstNumerical
  cisstMultiTask
  cisstParameterTypes
  cisstCommonXML)

find_package (cisst 1.2.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib
                 ${LIBUSB_LIBRARY_DIRS})
if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  if (NOT CISST_HAS_JSON)
    message( FATAL_ERROR "mtsOmegaHapticDevice requires cisst to be built with JSON" )
  endif ()

  # catkin/ROS paths
  cisst_set_output_path ()

  set (mtsOmegaHapticDevice_INCLUDE_DIR
    "${mtsOmegaHapticDevice_SOURCE_DIR}/include"
    "${mtsOmegaHapticDevice_BINARY_DIR}/include")
  set (mtsOmegaHapticDevice_HEADER_DIR "${mtsOmegaHapticDevice_SOURCE_DIR}/include/mtsOmegaHapticDevice")
  set (mtsOmegaHapticDevice_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (mtsOmegaHapticDevice_LIBRARIES mtsOmegaHapticDevice)

  include_directories (BEFORE ${mtsOmegaHapticDevice_HEADER_DIR}
  ${LIBUSB_INCLUDE_DIRS})

  # add all config files for this component
  cisst_add_config_files (mtsOmegaHapticDevice)

  set (mtsOmegaHapticDevice_HEADER_FILES
    "${mtsOmegaHapticDevice_HEADER_DIR}/dhdc.h"
    "${mtsOmegaHapticDevice_HEADER_DIR}/drdc.h"
    "${mtsOmegaHapticDevice_HEADER_DIR}/Robot.h"
    "${mtsOmegaHapticDevice_HEADER_DIR}/Transform.h"
    "${mtsOmegaHapticDevice_HEADER_DIR}/mtsTeleoperation.h"
    "${mtsOmegaHapticDevice_HEADER_DIR}/mtsOmegaHapticDeviceExport.h"
    ${mtsOmegaHapticDevice_CISST_DG_HDRS})

  set (mtsOmegaHapticDevice_SOURCE_FILES
    code/mtsTeleoperation.cpp
    ${mtsOmegaHapticDevice_CISST_DG_SRCS})

  add_library (
    mtsOmegaHapticDevice
    ${IS_SHARED}
    ${mtsOmegaHapticDevice_HEADER_FILES}
    ${mtsOmegaHapticDevice_SOURCE_FILES}
    )
    
  target_link_libraries(mtsOmegaHapticDevice Eigen3::Eigen  dhd drd ${LIBUSB_LIBRARIES}) 
  set_target_properties (
    mtsOmegaHapticDevice PROPERTIES
    VERSION ${mtsOmegaHapticDevice_VERSION}
    FOLDER "mtsOmegaHapticDevice")

  cisst_target_link_libraries (
    mtsOmegaHapticDevice
    ${REQUIRED_CISST_LIBRARIES}
   )


  install(
    DIRECTORY
      ${mtsOmegaHapticDevice_HEADER_DIR}
    DESTINATION include
    COMPONENT mtsOmegaHapticDevice-dev
  )

  install (
    TARGETS mtsOmegaHapticDevice
    COMPONENT mtsOmegaHapticDevice
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)


else (cisst_FOUND_AS_REQUIRED)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
