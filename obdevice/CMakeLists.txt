cmake_minimum_required (VERSION 2.6)
include(FindPkgConfig)

############################
# General project settings #
############################

PROJECT(OBDEVICE)
SET(OBDEVICE_VERSION_MAJOR 0)
SET(OBDEVICE_VERSION_MINOR 1)

FIND_PACKAGE(Eigen REQUIRED)

# source list
SET(SOURCES
    Kinect.cpp
    KinectPlayback.cpp
    UvcCam.cpp
    UvcVirtualCam.cpp
    ParentDevice3D.cpp
    CloudFactory.cpp
)

# if does exists find libopenni2 directories and libs
EXECUTE_PROCESS(COMMAND pkg-config libopenni2 --exists RESULT_VARIABLE RET_CODE)
IF(${RET_CODE} STREQUAL 0)
    MESSAGE(STATUS "Build with OpenNI2 ...")
    PKG_CHECK_MODULES(OPENNI libopenni2)
    INCLUDE_DIRECTORIES(${OPENNI_INCLUDE_DIRS})
    SET(SOURCES ${SOURCES} OpenNiDevice.cpp)
ENDIF()

#FIND_PACKAGE(PCL 1.6 REQUIRED)
#IF(PCL_FOUND)
#     SET(SOURCES ${SOURCES} PclCloudInterface.cpp)
#     MESSAGE(STATUS "Build with PCL interface ...")
#     INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
#     ADD_DEFINITIONS(${PCL_DEFINITIONS})
#     LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
#ENDIF()

INCLUDE_DIRECTORIES(/usr/include/openni
                    /usr/include/ni
                    $ENV{OBVIOUSLY_ROOT}
                    ../obcore/math
                    ../obcore
                    ${EIGEN_INCLUDE_DIRS}
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

####################
##### Packaging ####
####################
IF(CMAKE_BUILD_TYPE MATCHES Release)
INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" DESTINATION ${OBVIOUSLY_INC_DIR} FILES_MATCHING PATTERN "*.h")
INSTALL(TARGETS obdevice ARCHIVE DESTINATION ${OBVIOUSLY_LIB_DIR})
ENDIF()