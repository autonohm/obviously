cmake_minimum_required(VERSION 2.6)
project(math_test)

# Setup testing
add_subdirectory(../gtest-1.7.0 ../gtest-1.7.0/bin)
enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR} $ENV{OBVIOUSLY_ROOT})
link_directories($ENV{OBVIOUSLY_ROOT}/build/release/obcore
                 $ENV{OBVIOUSLY_ROOT}/build/release/obdevice
                 $ENV{OBVIOUSLY_ROOT}/build/release/obgraphic
                 )

# experimental
find_package(Eigen REQUIRED)
find_package(Qt4 REQUIRED)
find_package(VTK REQUIRED)

include(${QT_USE_FILE})
include(${VTK_USE_FILE})

add_definitions(${QT_DEFINITIONS})

include_directories(${Eigen_INCLUDE_DIRS})



# Add test cpp file
add_executable(runMatrixTest
    math/MatrixTest.cpp
)

add_executable(runQuaternionTest
    math/QuaternionTest.cpp
)

#add_executable(eigen-vs-gsl
#               base/eigen-vs-gsl.cpp
#               )

#add_executable(pointcloud
#               base/pointcloud.cpp
#               )


# Link test executable against gtest & gtest_main
target_link_libraries(runMatrixTest gtest gtest_main obcore gsl gslcblas)

target_link_libraries(runQuaternionTest gtest gtest_main obcore gsl gslcblas)

#target_link_libraries(eigen-vs-gsl
#                      obcore
#                      gsl
#                      gslcblas
#                      )

#target_link_libraries(pointcloud
#                      obcore
#                      obdevice
#                      obgraphic
#                      ${QT_LIBRARIES}
#                      QVTK
#                      ${VTK_LIBRARIES}
#                      gsl
#                      gslcblas
#                      )

add_test(
    NAME runMatrixTest
    COMMAND runMatrixTest
)

add_test(
    NAME runQuaternionTest
    COMMAND runQuaternionTest
)