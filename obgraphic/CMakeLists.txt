cmake_minimum_required (VERSION 2.6)

############################
# General project settings #
############################

PROJECT(OBGRAPHIC)
SET(OBCORE_VERSION_MAJOR 0)
SET(OBCORE_VERSION_MINOR 1)

FIND_PACKAGE(VTK REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Qt4)

INCLUDE(${VTK_USE_FILE})

SET(SOURCE
    Obvious2D.cpp
    Obvious2DMap.cpp
    Obvious3D.cpp
    VtkCloud.cpp
    )

# Qt part
IF(QT_FOUND)
  INCLUDE(${QT_USE_FILE})
  ADD_DEFINITIONS(${QT_DEFINITIONS})

  QT4_WRAP_CPP(MOC
               CloudWidget.h
               )

  SET(SOURCE
      ${SOURCE}
      CloudWidget.cpp
      ${MOC}
      )
ENDIF()

LINK_LIBRARIES(${VTK_LIBRARIES})

INCLUDE_DIRECTORIES(..
                    ${EIGEN_INCLUDE_DIRS}
                    )

add_library(obgraphic
            ${SOURCE}
            )

####################
##### Packaging ####
####################
IF(CMAKE_BUILD_TYPE MATCHES Release)
INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" DESTINATION ${OBVIOUSLY_INC_DIR} FILES_MATCHING PATTERN "*.h")
INSTALL(TARGETS obgraphic ARCHIVE DESTINATION ${OBVIOUSLY_LIB_DIR})
ENDIF()