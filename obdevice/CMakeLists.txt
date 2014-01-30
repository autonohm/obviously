cmake_minimum_required (VERSION 2.6)
include(FindPkgConfig)

############################
# General project settings #
############################

PROJECT(OBDEVICE)
SET(OBDEVICE_VERSION_MAJOR 0)
SET(OBDEVICE_VERSION_MINOR 1)

PKG_CHECK_MODULES(XML libxml++-2.6)

# source list
SET(SOURCES
    Kinect.cpp
    KinectPlayback.cpp
    UvcCam.cpp
    UvcVirtualCam.cpp
    OpenNIDevice.cpp
    Xtion.cpp
    ParentDevice3D.cpp
)

# if does exists find libopenni2 directories and libs
EXECUTE_PROCESS(COMMAND pkg-config libopenni2 --exists RESULT_VARIABLE RET_CODE)
IF(${RET_CODE} STREQUAL 0)
    MESSAGE(STATUS "Build with OpenNI2 ...")
    PKG_CHECK_MODULES(OPENNI libopenni2)
    INCLUDE_DIRECTORIES(${OPENNI_INCLUDE_DIRS})
    SET(SOURCES ${SOURCES} OpenNiDevice.cpp)
ENDIF()

INCLUDE_DIRECTORIES(/usr/include/openni
                    /usr/include/ni
                    $ENV{OBVIOUSLY_ROOT}
                    ../obcore/math
                    ../obcore
                    $ENV{OBVIOUSLY_ROOT}/../obviously_intern/thermo_cam/include
                    ${XML_INCLUDE_DIRS}
                    )

# check if necessary files for PMD are installed
IF(EXISTS /usr/local/include/PMDSDK)
    MESSAGE(STATUS "Build PMD stuff ...")
    INCLUDE_DIRECTORIES(/usr/local/include/PMDSDK)
    SET(SOURCES ${SOURCES} CamNano.cpp)
ENDIF()

# check if necessary files for sick lidar installed
IF(EXISTS /usr/local/include/LMS1xx.h)
    message(STATUS "Build Sick stuff ...")
    set(SOURCES ${SOURCES} SickLMS100.cpp)
ENDIF()

# Disabled unknown pragmas and reordering due to OpenNI wrapper. Remove option, if wrapper is not used anymore
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__OBVIOUSLY__ -Wno-unknown-pragmas -Wno-reorder")

ADD_LIBRARY(obdevice ${SOURCES})
