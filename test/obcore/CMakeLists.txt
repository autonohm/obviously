cmake_minimum_required(VERSION 2.6)
project(math_test)

# Setup testing
# add_subdirectory(../gtest-1.7.0 ../gtest-1.7.0/bin)
# enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR} $ENV{OBVIOUSLY_ROOT})
link_directories($ENV{OBVIOUSLY_ROOT}/build/debug/obcore)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

# Add test cpp file
#add_executable(runMatrixTest
#    math/MatrixTest.cpp
#)

add_executable(eigen-vs-gsl
               base/eigen-vs-gsl.cpp
               )

add_executable(pointcloud
               base/pointcloud.cpp
               )


# Link test executable against gtest & gtest_main
# target_link_libraries(runMatrixTest gtest gtest_main obcore gsl gslcblas)

target_link_libraries(eigen-vs-gsl
                      obcore
                      gsl
                      gslcblas
                      )

#add_test(
#    NAME runMatrixTest
#    COMMAND runMatrixTest
#)
