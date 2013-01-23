cmake_minimum_required (VERSION 2.6)

############################
# General project settings #
############################

PROJECT(OBGRAPHIC)
SET(OBCORE_VERSION_MAJOR 0)
SET(OBCORE_VERSION_MINOR 1)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

LINK_LIBRARIES(${VTK_LIBRARIES})

INCLUDE_DIRECTORIES(..)

add_library(obgraphic
	Obvious2D.cpp
	Obvious2DMap.cpp
	Obvious3D.cpp
	VtkCloud.cpp
	)
