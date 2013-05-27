cmake_minimum_required (VERSION 2.6)
include(FindPkgConfig)

############################
# General project settings #
############################

PROJECT(OBCORE)
SET(OBCORE_VERSION_MAJOR 0)

PKG_CHECK_MODULES(XML libxml++-2.6)

INCLUDE_DIRECTORIES(../..
                    ..
                    .
                    /usr/include/
                    ${XML_INCLUDE_DIRS}
                    )

add_library(obcore
	base/CartesianCloud.cpp
	base/CartesianCloudFactory.cpp
	base/Timer.cpp
	base/Logger.cpp
	base/tools.cpp
   	base/Cloud.cpp
	math/geometry.cpp
	math/Matrix.cpp
   	math/PID_Controller.cpp
   	math/MatD.cpp
   	math/MatRGB.cpp
   	math/VecD.cpp
   	math/VectorRefD.cpp
	filter/EuclideanFilter.cpp
	filter/CartesianFilter.cpp
	filter/NormalFilter.cpp
	filter/BoundingBoxFilter.cpp
   	grid/Grid2D.cpp
   	grid/GradientGrid.cpp
   	grid/HeightGrid.cpp
   	grid/ObstacleGrid.cpp
	)
