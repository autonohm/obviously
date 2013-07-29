#include "obgpu/Device.h"

#include <cuda_runtime_api.h>

#include <iostream>

namespace obvious { namespace gpu {

int Device::getDeviceCount(void)
{
    int count;
    cudaGetDeviceCount(&count);

    return count;
}

void Device::printDevices(void)
{
    const int kb = 1024;
    const int mb = kb * kb;

    std::cout << "CUDA Devices:" << std::endl;
    std::cout << "-----------------------------------------------" << std::endl;
    std::cout << "looking for devices ..." << std::endl;

    const int count = Device::getDeviceCount();

    std::cout << "found " << count << " devices." << std::endl << std::endl;

    for (int i = 0; i < count; i++)
    {
        cudaDeviceProp props;
        cudaGetDeviceProperties(&props, i);

        std::cout << i << ": " << props.name << ": " << props.major << "." << props.minor << std::endl;
        std::cout << "  Global memory:   " << props.totalGlobalMem / mb << "mb" << std::endl;
        std::cout << "  Shared memory:   " << props.sharedMemPerBlock / kb << "kb" << std::endl;
        std::cout << "  Constant memory: " << props.totalConstMem / kb << "kb" << std::endl;
        std::cout << "  Block registers: " << props.regsPerBlock << std::endl << std::endl;

        std::cout << "  Warp size:         " << props.warpSize << std::endl;
        std::cout << "  Threads per block: " << props.maxThreadsPerBlock << std::endl;
        std::cout << "  Max block dimensions: [ " << props.maxThreadsDim[0] << ", " << props.maxThreadsDim[1]  << ", " << props.maxThreadsDim[2] << " ]" << std::endl;
        std::cout << "  Max grid dimensions:  [ " << props.maxGridSize[0] << ", " << props.maxGridSize[1]  << ", " << props.maxGridSize[2] << " ]" << std::endl;
        std::cout << std::endl;
    }
}

} // end namespace gpu

} // end namespace obvious
