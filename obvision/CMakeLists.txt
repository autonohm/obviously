cmake_minimum_required (VERSION 2.6)

############################
# General project settings #
############################

PROJECT(OBVISION)
SET(OBVISION_VERSION_MAJOR 0)
SET(OBVISION_VERSION_MINOR 1)

INCLUDE_DIRECTORIES(..
					/usr/include/eigen3)

add_library(obvision
	icp/assign/PairAssignment.cpp
	icp/assign/AnnPairAssignment.cpp
	icp/assign/FlannPairAssignment.cpp
#	icp/assign/NaboPairAssignment.cpp
	icp/assign/ProjectivePairAssignment.cpp
	icp/assign/filter/ProjectionFilter.cpp
	icp/assign/filter/OcclusionFilter.cpp
	icp/assign/filter/TrimmedFilter.cpp
	icp/assign/filter/ReciprocalFilter.cpp
	icp/assign/filter/DistanceFilter.cpp
	icp/assign/filter/OutOfBoundsFilter2D.cpp
	icp/assign/filter/OutOfBoundsFilter3D.cpp
	icp/assign/filter/RobotFootprintFilter3D.cpp
	icp/ClosedFormEstimator2D.cpp
	icp/PointToPointEstimator3D.cpp
	icp/PointToPlaneEstimator3D.cpp
	icp/PointToLineEstimator2D.cpp
	icp/Icp.cpp
	icp/IcpTrace.cpp
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
	reconstruct/grid/TsdGridLocalization.cpp
	reconstruct/space/SensorPolar3D.cpp
	reconstruct/space/SensorProjective3D.cpp
	reconstruct/space/SensorPolar3D.cpp
	reconstruct/space/TsdSpace.cpp
	reconstruct/space/TsdSpaceComponent.cpp
	reconstruct/space/TsdSpacePartition.cpp
	reconstruct/space/TsdSpaceBranch.cpp
	reconstruct/space/RayCast3D.cpp
	reconstruct/space/RayCastAxisAligned3D.cpp
	#reconstruct/space/RayCastBackProjection3D.cpp
	)
