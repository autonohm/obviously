cmake_minimum_required (VERSION 2.6)
include(FindPkgConfig)

############################
# General project settings #
############################

PROJECT(OBCORE)
SET(OBCORE_VERSION_MAJOR 0)
SET(USE_EIGEN 1)

PKG_CHECK_MODULES(XML libxml++-2.6)

INCLUDE_DIRECTORIES(../..
                    ..
                    .
                    /usr/include/
                    ${XML_INCLUDE_DIRS}
                    /usr/include/eigen3
                    )

SET(LIBRARY     base/CartesianCloud.cpp
                base/CartesianCloudFactory.cpp
                base/Timer.cpp
                base/Logger.cpp
                base/tools.cpp
                math/geometry.cpp
                math/linalg/MatrixFactory.cpp 
                math/PID_Controller.cpp
                math/IntegratorSimpson.cpp
                math/TransformationWatchdog.cpp
                filter/EuclideanFilter.cpp
                filter/CartesianFilter.cpp
                filter/NormalFilter.cpp
                filter/BoundingBoxFilter.cpp
                )
              
IF(USE_EIGEN)
  message("Using libeigen for obcore")
  add_definitions(-DEIGENUSED)
  #SET(EIGENUSED 1)
  #set(VAR ${VAR} PARENT_SCOPE);
  SET(EIGENUSED 1 CACHE INTEGER "Eigen Used")
  SET(LIBRARY ${LIBRARY} math/linalg/eigen/Matrix.cpp
                         math/linalg/eigen/Vector.cpp
                         )
ELSE()
  message("Using gsl for obcore")
  SET(LIBRARY ${LIBRARY} math/linalg/gsl/Matrix.cpp
                         math/linalg/gsl/Vector.cpp
                         )
ENDIF()

add_library(obcore ${LIBRARY})

