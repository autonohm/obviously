cmake_minimum_required (VERSION 2.6)

############################
# General project settings #
############################

PROJECT(OBVISION)
SET(OBVISION_VERSION_MAJOR 0)
SET(OBVISION_VERSION_MINOR 1)

add_compile_options(-std=c++11)

INCLUDE_DIRECTORIES(..
					/usr/include/eigen3)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(obvision STATIC
	registration/icp/assign/PairAssignment.cpp
	registration/icp/assign/AnnPairAssignment.cpp
	registration/icp/assign/FlannPairAssignment.cpp
#	registration/icp/assign/NaboPairAssignment.cpp
	registration/icp/assign/ProjectivePairAssignment.cpp
	registration/icp/assign/filter/ProjectionFilter.cpp
	registration/icp/assign/filter/OcclusionFilter.cpp
	registration/icp/assign/filter/TrimmedFilter.cpp
	registration/icp/assign/filter/ReciprocalFilter.cpp
	registration/icp/assign/filter/DistanceFilter.cpp
	registration/icp/assign/filter/OutOfBoundsFilter2D.cpp
	registration/icp/assign/filter/OutOfBoundsFilter3D.cpp
	registration/icp/assign/filter/RobotFootprintFilter.cpp
	registration/icp/ClosedFormEstimator2D.cpp
	registration/icp/PointToPointEstimator3D.cpp
	registration/icp/PointToPlaneEstimator3D.cpp
	registration/icp/PointToLineEstimator2D.cpp
	registration/icp/Icp.cpp
	registration/icp/IcpMultiInitIterator.cpp
	registration/ndt/Ndt.cpp
	registration/ransacMatching/RandomMatching.cpp
	registration/ransacMatching/TwinPointMatching.cpp
	registration/ransacMatching/RandomNormalMatching.cpp
	registration/ransacMatching/PDFMatching.cpp
	registration/ransacMatching/TSD_PDFMatching.cpp
	registration/Trace.cpp
	normals/NormalsEstimator.cpp
	mesh/TriangleMesh.cpp
	reconstruct/Sensor.cpp
	reconstruct/grid/SensorPolar2D.cpp
	reconstruct/grid/TsdGrid.cpp
	reconstruct/grid/TsdGridComponent.cpp
	reconstruct/grid/TsdGridPartition.cpp
	reconstruct/grid/TsdGridBranch.cpp
	reconstruct/grid/RayCastPolar2D.cpp
	reconstruct/grid/RayCastAxisAligned2D.cpp
	reconstruct/space/SensorPolar3D.cpp
	reconstruct/space/SensorProjective3D.cpp
	reconstruct/space/SensorPolar3D.cpp
	reconstruct/space/SensorVelodyne3D.cpp
	reconstruct/space/SensorPolar3DBase.cpp
    reconstruct/space/SensorPolar2DWith3DPose.cpp
	reconstruct/space/TsdSpace.cpp
	reconstruct/space/TsdSpaceComponent.cpp
	reconstruct/space/TsdSpacePartition.cpp
	reconstruct/space/TsdSpaceBranch.cpp
	reconstruct/space/RayCast3D.cpp
	reconstruct/space/RayCastAxisAligned3D.cpp
	#reconstruct/space/RayCastBackProjection3D.cpp
	planning/Obstacle.cpp
	planning/AStar.cpp
	planning/AStarMap.cpp
	planning/AStarNode.cpp
	ransac/RansacPrimitives.cpp
	)

target_link_libraries(obvision
                 ${PCL_LIBRARIES}
                 )

####################
##### Packaging ####
####################
IF(CMAKE_BUILD_TYPE MATCHES Release)
INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" DESTINATION ${OBVIOUSLY_INC_DIR} FILES_MATCHING PATTERN "*.h")
INSTALL(TARGETS obvision ARCHIVE DESTINATION ${OBVIOUSLY_LIB_DIR})
ENDIF()
