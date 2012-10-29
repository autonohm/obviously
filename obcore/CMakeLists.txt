cmake_minimum_required (VERSION 2.6)
include(FindPkgConfig)

############################
# General project settings #
############################

PROJECT(OBCORE)
SET(OBCORE_VERSION_MAJOR 0)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")

PKG_CHECK_MODULES(XML libxml++-2.6)

INCLUDE_DIRECTORIES(../..
                    ..
                    /usr/include/
                    ${XML_INCLUDE_DIRS}
                    )

add_library(obcore
	base/CartesianCloud.cpp
	base/CartesianCloudFactory.cpp
	base/Timer.cpp
	base/Logger.cpp
	base/tools.cpp
	math/geometry.cpp
	math/Matrix.cpp
        math/MatD.cpp
        math/MatRGB.cpp
	)

# Braucht man glaube ich hier noch gar nicht?! (Knueppl)
target_link_libraries(obcore
                      ${XML_LIBRARIES}
                      )
