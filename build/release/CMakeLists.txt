cmake_minimum_required(VERSION 2.6)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp -Wall -O2")

ADD_SUBDIRECTORY(../../obcore obcore)
ADD_SUBDIRECTORY(../../obdevice obdevice)
ADD_SUBDIRECTORY(../../obgraphic obgraphic)
ADD_SUBDIRECTORY(../../obvision obvision)
ADD_SUBDIRECTORY(../../applications applications)
