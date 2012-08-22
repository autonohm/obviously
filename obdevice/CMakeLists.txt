cmake_minimum_required (VERSION 2.6)

############################
# General project settings #
############################

PROJECT(OBDEVICE)
SET(OBDEVICE_VERSION_MAJOR 0)
SET(OBDEVICE_VERSION_MINOR 1)

INCLUDE_DIRECTORIES(/usr/include/openni /usr/include/ni $ENV{OBVIOUSLY_ROOT})

ADD_LIBRARY(obdevice
	Kinect.cpp
	KinectPlayback.cpp
	UvcCam.cpp
	UvcVirtualCam.cpp
	)