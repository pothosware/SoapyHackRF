######################################################################## 
 # Build Soapy SDR support module for HackRF
 ######################################################################## 

cmake_minimum_required(VERSION 2.8.7)
project(SoapyHackRF CXX)

find_package(SoapySDR NO_MODULE) 
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


include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${LIBHACKRF_INCLUDE_DIRS})

if(CMAKE_COMPILER_IS_GNUCXX)
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -std=c++11")
endif()

SOAPY_SDR_MODULE_UTIL(
    TARGET HackRFSupport
    SOURCES
	HackRF_Registation.cpp
	HackRF_Settings.cpp
	HackRF_Streaming.cpp
    LIBRARIES ${LIBHACKRF_LIBRARIES}
)
