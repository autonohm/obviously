cmake_minimum_required (VERSION 2.6)

############################
# General project settings #
############################

PROJECT(OBCORE)
SET(OBCORE_VERSION_MAJOR 0)
SET(OBCORE_VERSION_MINOR 1)

INCLUDE_DIRECTORIES(../.. ..)

add_library(obcore
	base/CartesianCloud.cpp
	base/CartesianCloudFactory.cpp
	base/Timer.cpp
	base/tools.cpp
	math/geometry.cpp
	math/Matrix.cpp
	)
