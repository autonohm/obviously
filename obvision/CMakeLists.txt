cmake_minimum_required (VERSION 2.6)

############################
# General project settings #
############################

PROJECT(OBVISION)
SET(OBVISION_VERSION_MAJOR 0)
SET(OBVISION_VERSION_MINOR 1)

INCLUDE_DIRECTORIES(..)

add_library(obvision
	icp/assign/PairAssignment.cpp
	icp/assign/AnnPairAssignment.cpp
	icp/assign/FlannPairAssignment.cpp
	icp/assign/ProjectivePairAssignment.cpp
	icp/assign/filter/ProjectionFilter.cpp
	icp/assign/filter/OcclusionFilter.cpp
	icp/assign/filter/TrimmedFilter.cpp
	icp/assign/filter/ReciprocalFilter.cpp
	icp/assign/filter/SubsamplingFilter.cpp
	icp/assign/filter/DistanceFilter.cpp
	icp/ClosedFormEstimator2D.cpp
	icp/PointToPointEstimator3D.cpp
	icp/PointToPlaneEstimator3D.cpp
	icp/Icp.cpp
	normals/NormalsEstimator.cpp
	mesh/TriangleMesh.cpp
	reconstruct/TsdGrid.cpp
	reconstruct/SensorPolar2D.cpp
	reconstruct/RayCast2D.cpp
	)
