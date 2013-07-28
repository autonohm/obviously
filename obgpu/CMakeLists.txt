CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

FIND_PACKAGE(CUDA)

INCLUDE(FindCUDA)

SET(DEVICE_SRC $ENV{OBVIOUSLY_ROOT}/obgpu/device)
#add_definitions(-std=gnu++0x)


INCLUDE_DIRECTORIES($ENV{OBVIOUSLY_ROOT})

CUDA_ADD_LIBRARY(obgpu
                 ${DEVICE_SRC}/PointCloud.cu
                 )

# if you need to pass options/flags
set( custom_cuda_flags "-arch=sm_20" CACHE STRING "Set the cuda flags like --arch=sm_20 --arch=sm_13 --ptxas-options=-v --use_fast_math")
list(APPEND CUDA_NVCC_FLAGS ${custom_cuda_flags})

list(APPEND CUDA_NVCC_FLAGS -Xcompiler;-fopenmp;-lgomp)
#here -Xcompiler means that the flag that fallow is for GCC and not nvcc
