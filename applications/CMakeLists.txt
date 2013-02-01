cmake_minimum_required (VERSION 2.6)
include(FindPkgConfig)

############################
# General project settings #
############################

PROJECT(APPLICATIONS)
SET(APPLICATIONS_VERSION_MAJOR 0)
SET(APPLICATIONS_VERSION_MINOR 1)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
PKG_CHECK_MODULES(XML libxml++-2.6)

INCLUDE_DIRECTORIES(/usr/include/openni
                    /usr/include/ni
                    $ENV{OBVIOUSLY_ROOT}
                    /usr/include/qt4/
                    /usr/include/lua5.1
                    /opt/ros/electric/stacks/perception_pcl/flann/include/
                    ../obcore/math
                    ../obcore/filter
                    ../obcore
                    ${XML_INCLUDE_DIRS}
                    )

# check if necessary files for PMD are installed
if(EXISTS /usr/include/PMDSDK)
	INCLUDE_DIRECTORIES(/usr/include/PMDSDK)
	LINK_DIRECTORIES(/home/christian/workspace/CamBoardNano/PMDSDK/lib)
	MESSAGE("Found PMDSDK. Will build driver for CamBoard Nano.")
	#ADD_EXECUTABLE(nanoStream nanoStream.cpp)
	#TARGET_LINK_LIBRARIES(nanoStream obgraphic obdevice obcore vtkHybrid gsl gslcblas -lpmdaccess2 ${XML_LIBRARIES})
	#ADD_EXECUTABLE(nanoStreamIR nanoStreamIR.cpp)
   #TARGET_LINK_LIBRARIES(nanoStreamIR obgraphic obdevice obcore v4l2 glut GL -lpmdaccess2)
endif()

LINK_DIRECTORIES(/opt/ros/electric/stacks/perception_pcl/flann/lib64/)

ADD_EXECUTABLE(obvious3D_show obvious3D_show.cpp)
ADD_EXECUTABLE(obvious3D_map obvious3D_map.cpp)
ADD_EXECUTABLE(kinect_perspective kinect_perspective.cpp)
ADD_EXECUTABLE(icp_matching2D icp_matching2D.cpp)
ADD_EXECUTABLE(icp_interactive3D icp_interactive3D.cpp)
ADD_EXECUTABLE(kinect_stream_show kinect_stream_show.cpp)
ADD_EXECUTABLE(kinect_mesh_show kinect_mesh_show.cpp)
ADD_EXECUTABLE(kinect_playback kinect_playback.cpp)
ADD_EXECUTABLE(kinect_localize kinect_localize.cpp)
ADD_EXECUTABLE(uvccam_serialize uvccam_serialize.cpp)
ADD_EXECUTABLE(uvcvirtualcam_serialize uvcvirtualcam_serialize.cpp)
ADD_EXECUTABLE(uvccam_stream uvccam_stream.cpp)
ADD_EXECUTABLE(uvccam_querycapabilities uvccam_querycapabilities.cpp)
ADD_EXECUTABLE(uvccam_finddevice uvccam_finddevice.cpp)
ADD_EXECUTABLE(synthetic_pointcloud synthetic_pointcloud.cpp)
ADD_EXECUTABLE(read_lua_config read_lua_config.cpp)
ADD_EXECUTABLE(logging_example logging_example.cpp)
#ADD_EXECUTABLE(tsd_test tsd_test.cpp)
ADD_EXECUTABLE(showCloud showCloud.cpp)
ADD_EXECUTABLE(filterTest filter_test.cpp)
#ADD_EXECUTABLE(cloud2grid  cloud2grid.cpp)
ADD_EXECUTABLE(xtionStream xtionStream.cpp)
ADD_EXECUTABLE(obstacleView obstacleView.cpp)

SET(CORELIBS obcore gsl gslcblas)
SET(DEVICELIBS obdevice OpenNI v4l2 udev)
SET(GRAPHICLIBS obgraphic vtkHybrid glut GL jpeg)
SET(VISIONLIBS obvision ann flann)

TARGET_LINK_LIBRARIES(obvious3D_show ${GRAPHICLIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(obvious3D_map ${GRAPHICLIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(kinect_perspective ${GRAPHICLIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(icp_matching2D ${VISIONLIBS} ${CORELIBS} ${XML_LIBRARIES})
TARGET_LINK_LIBRARIES(icp_interactive3D ${GRAPHICLIBS} ${VISIONLIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(kinect_stream_show ${VISIONLIBS} ${GRAPHICLIBS} ${DEVICELIBS} ${CORELIBS} ${XML_LIBRARIES})
TARGET_LINK_LIBRARIES(kinect_mesh_show ${VISIONLIBS} ${GRAPHICLIBS} ${DEVICELIBS} ${CORELIBS} ${XML_LIBRARIES})
TARGET_LINK_LIBRARIES(kinect_playback ${GRAPHICLIBS} ${DEVICELIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(kinect_localize ${VISIONLIBS} ${GRAPHICLIBS} ${DEVICELIBS} ${CORELIBS} ${XML_LIBRARIES})
TARGET_LINK_LIBRARIES(uvccam_serialize ${DEVICELIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(uvcvirtualcam_serialize ${DEVICELIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(uvccam_stream ${GRAPHICLIBS} ${DEVICELIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(uvccam_querycapabilities ${DEVICELIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(uvccam_finddevice ${DEVICELIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(synthetic_pointcloud ${GRAPHICLIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(read_lua_config lua5.1)
TARGET_LINK_LIBRARIES(logging_example obcore)
#TARGET_LINK_LIBRARIES(tsd_test ${VISIONLIBS} ${GRAPHICLIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(showCloud ${GRAPHICLIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(filterTest ${GRAPHICLIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(xtionStream ${VISIONLIBS} ${GRAPHICLIBS} ${DEVICELIBS} ${CORELIBS} ${XML_LIBRARIES})
#TARGET_LINK_LIBRARIES(cloud2grid ${GRAPHICLIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(obstacleView ${GRAPHICLIBS} ${DEVICELIBS} ${CORELIBS} ${XML_LIBRARIES})



