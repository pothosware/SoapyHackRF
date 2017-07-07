# - Try to find libhackrf
# Once done this will define
#  LIBHACKRF_FOUND - System has libhackrf
#  LIBHACKRF_INCLUDE_DIRS - The libhackrf include directories
#  LIBHACKRF_LIBRARIES - The libraries needed to use libhackrf

find_package(PkgConfig)
pkg_check_modules(PC_LIBHACKRF QUIET libhackrf)

find_path(LIBHACKRF_INCLUDE_DIR
    NAMES hackrf.h
    HINTS
        $ENV{LIBHACKRF_DIR}/include
        ${PC_LIBHACKRF_INCLUDEDIR}
        ${PC_LIBHACKRF_INCLUDE_DIRS}
    PATH_SUFFIXES libhackrf
)

find_library(LIBHACKRF_LIBRARY
    NAMES hackrf
    HINTS
        $ENV{LIBHACKRF_DIR}/lib
        ${PC_LIBHACKRF_LIBDIR}
        ${PC_LIBHACKRF_LIBRARY_DIRS}
)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBHACKRF DEFAULT_MSG LIBHACKRF_LIBRARY LIBHACKRF_INCLUDE_DIR)

mark_as_advanced(LIBHACKRF_INCLUDE_DIR LIBHACKRF_LIBRARY)

set(LIBHACKRF_INCLUDE_DIRS ${LIBHACKRF_INCLUDE_DIR})
set(LIBHACKRF_LIBRARIES ${LIBHACKRF_LIBRARY})
