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

SOAPY_SDR_MODULE_UTIL(
    TARGET HackRFSupport
    SOURCES
	HackRF_Registation.cpp
	HackRF_Settings.cpp
	HackRF_Streaming.cpp
    LIBRARIES ${LIBHACKRF_LIBRARIES}
)
