cmake_minimum_required(VERSION 2.6)

SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -DDEBUG")

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake)

ADD_SUBDIRECTORY(../../obcore obcore)
ADD_SUBDIRECTORY(../../obgraphic obgraphic)
ADD_SUBDIRECTORY(../../obvision obvision)
ADD_SUBDIRECTORY(../../obdevice obdevice)
ADD_SUBDIRECTORY(../../applications applications)
