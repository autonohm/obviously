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
	math/geometry.cpp
	math/Matrix.cpp
<<<<<<< HEAD
   math/MatD.cpp
   math/MatRGB.cpp
	filter/EuclidianFilterD.cpp
=======
        math/MatD.cpp
        math/MatRGB.cpp
        math/PID_Controller.cpp
>>>>>>> 98ebc6655181c628fbf8e55a5137d5bc6eb57c1c
	)

# Braucht man glaube ich hier noch gar nicht?! (Knueppl)
target_link_libraries(obcore
                      ${XML_LIBRARIES}
                      )
