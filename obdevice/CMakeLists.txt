cmake_minimum_required (VERSION 2.6)
include(FindPkgConfig)

############################
# General project settings #
############################

PROJECT(OBDEVICE)
SET(OBDEVICE_VERSION_MAJOR 0)
SET(OBDEVICE_VERSION_MINOR 1)

PKG_CHECK_MODULES(XML libxml++-2.6)

INCLUDE_DIRECTORIES(/usr/include/openni
                    /usr/include/ni
                    $ENV{OBVIOUSLY_ROOT}
                    ../obcore/math
                    ../obcore
                    $ENV{OBVIOUSLY_ROOT}/../obviously_intern/thermo_cam/include
                    ${XML_INCLUDE_DIRS}
                    )

# check if necessary files for PMD are installed
if(EXISTS /usr/include/PMDSDK)
	INCLUDE_DIRECTORIES(/usr/include/PMDSDK)
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__OBVIOUSLY__")

ADD_LIBRARY(obdevice
	Kinect.cpp
	KinectPlayback.cpp
	UvcCam.cpp
	UvcVirtualCam.cpp
# uncomment this to include the tof camera of PMD
#    CamNano.cpp
# uncomment this for include the thermo camera. I know that is not a realy nice solution...
#    KinectThermoDevice.cpp
    Xtion.cpp
    OpenNIDevice.cpp
    ParentDevice3D.cpp
# uncomment this to include the tof camera of PMD
    CameraCalibration.cpp
#	SickLMS100.cpp
        )



