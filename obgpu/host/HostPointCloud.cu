#include "obgpu/host/HostPointCloud.h"
#include "obgpu/device/DevicePointCloud.h"

namespace obvious { namespace gpu { namespace host {

HostPointCloud::~HostPointCloud(void)
{
    if (_data)
        cudaFree(_data);
}

void HostPointCloud::upload(const obvious::PointCloud<obvious::PointXyz>& cloud)
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

    cudaMemcpy(_data, cloud.points().data(), cloud.size() * sizeof(obvious::gpu::device::PointXyz), cudaMemcpyHostToDevice);
}

void HostPointCloud::download(obvious::PointCloud<obvious::PointXyz>& cloud)
{
    if (cloud.size() != _size)
        cloud.resize(_size);

    cudaMemcpy(cloud.points().data(), _data, cloud.size() * sizeof(obvious::gpu::device::PointXyz), cudaMemcpyDeviceToHost);
}

} // end namespace device

} // end namespace gpu

} // end namespace obvious
