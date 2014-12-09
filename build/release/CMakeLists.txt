cmake_minimum_required(VERSION 2.6)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp -Wall -O2 -DNDEBUG -funroll-loops -msse -mmmx -mtune=core2 -msse2 -msse3 -msse4 -mssse3")

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake)

####################
# Debian Packaging #
####################
SET(OBVIOUSLY_VERSION_MAJOR 1)
SET(OBVIOUSLY_VERSION_MINOR 0)
SET(OBVIOUSLY_VERSION_PATCH 0)

SET(OBVIOUSLY_LIB_DIR "usr/lib")
SET(OBVIOUSLY_INC_DIR "usr/include/obviously")
SET(OBVIOUSLY_EXAMPLES_DIR "usr/share/doc/libobviously/examples")
SET(OBVIOUSLY_RES_DIR "usr/share/libobviously")

SET(CPACK_PACKAGE_NAME "libobviously")
SET(CPACK_DEBIAN_PACKAGE_DESCRIPTION "Open Robotic Vision and Utilities Library")

SET(CPACK_PACKAGE_VENDOR "Technische Hochschule Nürnberg Georg Simon Ohm")
INCLUDE (InstallRequiredSystemLibraries)
SET(CPACK_DEBIAN_PACKAGE_VERSION)
SET(CPACK_PACKAGE_VERSION_MAJOR ${OBVIOUSLY_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${OBVIOUSLY_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${OBVIOUSLY_VERSION_PATCH})
SET(CPACK_PACKAGE_VERSION ${OBVIOUSLY_VERSION_MAJOR}.${OBVIOUSLY_VERSION_MINOR}.${OBVIOUSLY_VERSION_PATCH})

SET(CPACK_GENERATOR "DEB")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Stefan May <stefan.may@th-nuernberg.de>") #required
set(CPACK_SET_DESTDIR true)
SET(CPACK_INSTALL_PREFIX "")
EXECUTE_PROCESS ( COMMAND dpkg --print-architecture 
    OUTPUT_VARIABLE arch.output OUTPUT_STRIP_TRAILING_WHITESPACE)
IF(BUILD_ARM)
  SET(ARCH_TEXT_PACKAGE armhf) 
  SET(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "armhf")
ELSE()
  SET(ARCH_TEXT_PACKAGE ${arch.output})
  SET(CPACK_DEBIAN_PACKAGE_ARCHITECTURE ${arch.output})
ENDIF()
SET(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${ARCH_TEXT_PACKAGE}")


SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libgsl0-dev (>=1.15), libudev-dev(>=1.0), libxml++2.6-dev(>= 2.6.2), gcc (>= 4.6), build-essential (>= 1.14.18), cmake (>= 2.6), libv4l-dev (>=0.8)")

SET(CPACK_RESOURCE_FILE_LICENSE "../../License.txt")
IF(CPACK_GENERATOR STREQUAL "DEB")
CONFIGURE_FILE("fixup_deb.sh.in" "${CMAKE_CURRENT_BINARY_DIR}/fixup_deb.sh" @ONLY IMMEDIATE)
ENDIF()
#########################



ADD_SUBDIRECTORY(../../obcore obcore)
ADD_SUBDIRECTORY(../../obdevice obdevice)
ADD_SUBDIRECTORY(../../obgraphic obgraphic)
ADD_SUBDIRECTORY(../../obvision obvision)
ADD_SUBDIRECTORY(../../applications applications)
#ADD_SUBDIRECTORY(../../test/obcore test)

INCLUDE (CPack)
