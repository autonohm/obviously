cmake_minimum_required (VERSION 2.6)

############################
# General project settings #
############################

PROJECT(OBDEVICE)
SET(OBDEVICE_VERSION_MAJOR 0)
SET(OBDEVICE_VERSION_MINOR 1)

INCLUDE_DIRECTORIES(/usr/include/openni
                    /usr/include/ni
                    $ENV{OBVIOUSLY_ROOT}
                    ../obcore/math
                    ../obcore
                    $ENV{OBVIOUSLY_ROOT}/../thermo_cam/include
                    )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__OBVIOUSLY__")

ADD_LIBRARY(obdevice
	Kinect.cpp
	KinectPlayback.cpp
	UvcCam.cpp
	UvcVirtualCam.cpp
        Device3D.cpp
        KinectDevice.cpp
# uncomment this for include the thermo camera. I know that is not a realy nice solution...
#        ThermoDevice.cpp
	)
