#include "obgpu/PointCloud.h"
#include "obgpu/point-types.h"

#include <cuda_runtime_api.h>

namespace obvious { namespace gpu {
/*
__global__ void copyInlier(const PointXyz* source, const unsigned int n, const bool* inlier, PointXyz*& target)
{
    __device__ unsigned int tarIdx = 0;
    int i = threadIdx.x + blockIdx.x * blockDim.x;

    if (i < n)
    {
        if (inlier.data[i])
        {
            target[tarIdx] = source[i];
            ++tarIdx;
        }
    }
}
*/
PointCloud::PointCloud(const PointCloud& cloud, const bool* inlier)
    : _data(0),
      _type(cloud._type),
      _size(0)
{
    unsigned int counter = 0;

    for (unsigned int i = 0; i < cloud._size; ++i)
        if (inlier[i]) counter++;

    _size = counter;
    cudaMalloc(&_data, _size);

}

PointCloud::~PointCloud(void)
{
    if (_data)
        cudaFree(_data);
}

void PointCloud::upload(const obvious::PointCloud<obvious::PointXyz>& cloud)
{
    if (_data && _type != XYZ)
    {
        cudaFree(_data);
        _data = 0;
        _type = XYZ;
        _size = 0;
    }

    if (_size < cloud.size())
    {
        cudaFree(_data);
        cudaMalloc(&_data, cloud.size());
        _size = cloud.size();
    }
    else if (_size > cloud.size())
    {
        _size = cloud.size();
    }

    cudaMemcpy(_data, cloud.points().data(), cloud.size() * sizeof(obvious::gpu::PointXyz), cudaMemcpyHostToDevice);
}

void PointCloud::download(obvious::PointCloud<obvious::PointXyz>& cloud)
{
    if (cloud.size() != _size)
        cloud.resize(_size);

    cudaMemcpy(cloud.points().data(), _data, cloud.size() * sizeof(obvious::gpu::PointXyz), cudaMemcpyDeviceToHost);
}

} // end namespace gpu

} // end namespace obvious
