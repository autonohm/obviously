cmake_minimum_required (VERSION 2.6)
include(FindPkgConfig)

############################
# General project settings #
############################

PROJECT(OBCORE)

SET(USE_EIGEN 0)

FIND_PACKAGE(Eigen REQUIRED)

INCLUDE_DIRECTORIES(../..
                    ..
                    .
                    ${EIGEN_INCLUDE_DIRS}
                    )

SET(LIBRARY     base/CartesianCloud.cpp
                base/CartesianCloudFactory.cpp
                base/Timer.cpp
                base/Logger.cpp
                base/tools.cpp
                base/Time.cpp
                base/PointCloud.cpp
                statemachine/StateMachine.cpp
                statemachine/Agent.cpp
                statemachine/states/StateBase.cpp
                statemachine/states/StatePing.cpp
                statemachine/states/StatePong.cpp
                math/geometry.cpp
                math/linalg/MatrixFactory.cpp
                math/PID_Controller.cpp
                math/IntegratorSimpson.cpp
                math/TransformationWatchdog.cpp
                math/Trajectory.cpp
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

ADD_LIBRARY(obcore ${LIBRARY})

####################
##### Packaging ####
####################
IF(CMAKE_BUILD_TYPE MATCHES Release)
INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" DESTINATION ${OBVIOUSLY_INC_DIR} FILES_MATCHING PATTERN "*.h")
INSTALL(TARGETS obcore ARCHIVE DESTINATION ${OBVIOUSLY_LIB_DIR})
ENDIF()