######################################################################## 
 # Build Soapy SDR support module for HackRF
 ######################################################################## 

cmake_minimum_required(VERSION 2.8.7)
project(SoapyHackRF CXX)

find_package(SoapySDR "0.4.0" NO_MODULE)
 if (NOT SoapySDR_FOUND) 
    message(FATAL_ERROR "Soapy SDR development files not found...") 
 endif () 

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(LIBHACKRF)

if (NOT LIBHACKRF_FOUND) 
     message(FATAL_ERROR "HackRF development files not found...") 
endif () 
message(STATUS "LIBHACKRF_INCLUDE_DIRS - ${LIBHACKRF_INCLUDE_DIRS}")
message(STATUS "LIBHACKRF_LIBRARIES - ${LIBHACKRF_LIBRARIES}")

#version check for recent hackrf with device list API
message(STATUS "Checking for hackrf_device_list API...")
message(STATUS "  Reading ${LIBHACKRF_INCLUDE_DIRS}/hackrf.h...")
file(READ ${LIBHACKRF_INCLUDE_DIRS}/hackrf.h hackrf_h)
string(FIND "${hackrf_h}" "hackrf_device_list" has_hackrf_device_list)
if ("${has_hackrf_device_list}" STREQUAL "-1")
    message(FATAL_ERROR "  libhackrf too old, missing hackrf_device_list API")
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${LIBHACKRF_INCLUDE_DIRS})

#enable c++11 features
if(CMAKE_COMPILER_IS_GNUCXX)

    #C++11 is a required language feature for this project
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_STD_CXX11)
    if(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    else(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    endif()

    #disable warnings for unused parameters
    add_definitions(-Wno-unused-parameter)

endif(CMAKE_COMPILER_IS_GNUCXX)

if (APPLE)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wc++11-extensions")
endif(APPLE)

SOAPY_SDR_MODULE_UTIL(
    TARGET HackRFSupport
    SOURCES
	HackRF_Registration.cpp
	HackRF_Settings.cpp
	HackRF_Streaming.cpp
	HackRF_Session.cpp
    LIBRARIES ${LIBHACKRF_LIBRARIES}
)

########################################################################
# uninstall target
########################################################################
add_custom_target(uninstall
    "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

