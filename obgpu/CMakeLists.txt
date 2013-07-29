CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

FIND_PACKAGE(CUDA)

INCLUDE(FindCUDA)

SET(SRC_DIR $ENV{OBVIOUSLY_ROOT}/obgpu)

INCLUDE_DIRECTORIES($ENV{OBVIOUSLY_ROOT})

CUDA_ADD_LIBRARY(obgpu
                 ${SRC_DIR}/PointCloud.cu
                 ${SRC_DIR}/filter/PassThrough.cu
                 ${SRC_DIR}/filter/PassThrough.cpp
                 )

# if you need to pass options/flags
set( custom_cuda_flags "-arch=sm_20" CACHE STRING "Set the cuda flags like --arch=sm_20 --arch=sm_13 --ptxas-options=-v --use_fast_math")
list(APPEND CUDA_NVCC_FLAGS ${custom_cuda_flags})

list(APPEND CUDA_NVCC_FLAGS -Xcompiler;-fopenmp;-lgomp)
#here -Xcompiler means that the flag that fallow is for GCC and not nvcc
