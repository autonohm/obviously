#include "obgpu/Device.h"

#include <cuda_runtime_api.h>

#include <iostream>

namespace obvious { namespace gpu {

Device::Device(void)
    : d_source(0),
      d_inlier(0),
      h_sync(false)
{

}

Device::~Device(void)
{
    cudaFree(d_inlier);
}

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
        std::cout << "  Max block dimensions: [ " << props.maxThreadsDim[0] << ", " << props.maxThreadsDim[1]  << ", " << props.maxThreadsDim[2]
                  << " ]" << std::endl;
        std::cout << "  Max grid dimensions:  [ " << props.maxGridSize[0] << ", " << props.maxGridSize[1]  << ", " << props.maxGridSize[2] << " ]"
                  << std::endl;
        std::cout << std::endl;
    }
}

void Device::setSource(gpu::PointCloud* cloud)
{
    if (!d_source || !cloud || d_source->size() != cloud->size())
    {
        cudaFree(d_inlier);
        cudaMalloc(reinterpret_cast<void**>(&d_inlier), cloud->size() * sizeof(unsigned int));
    }

    d_source = cloud;
    h_sync = true;
}

const std::vector<unsigned int>& Device::inlier(void)
{
    if (h_sync)
        this->downloadInlier();

    return h_inlier;
}

void Device::downloadInlier(void)
{
    bool inlier[d_source->size()];

    cudaMemcpy(inlier, d_inlier, d_source->size() * sizeof(bool), cudaMemcpyDeviceToHost);
    h_inlier.clear();
    h_sync = false;

    for (unsigned int i = 0; i < d_source->size(); ++i)
        if (inlier[i]) h_inlier.push_back(i);
}

} // end namespace gpu

} // end namespace obvious
