cmake_minimum_required (VERSION 2.6)

############################
# General project settings #
############################

PROJECT(OBGRAPHIC)
SET(OBCORE_VERSION_MAJOR 0)
SET(OBCORE_VERSION_MINOR 1)

FIND_PACKAGE(VTK REQUIRED)
FIND_PACKAGE(Qt4)
FIND_PACKAGE(Eigen)

INCLUDE(${VTK_USE_FILE})

# Qt part
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

QT4_WRAP_CPP(MOC
             CloudWidget.h
             )


LINK_LIBRARIES(${VTK_LIBRARIES})

INCLUDE_DIRECTORIES(..
                    ${Eigen_INCLUDE_DIRS}
                    )

add_library(obgraphic
	Obvious2D.cpp
	Obvious2DMap.cpp
	Obvious3D.cpp
	VtkCloud.cpp
        CloudWidget.cpp
        ${MOC}
	)
