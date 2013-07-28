#include "PointCloud.h"

namespace obvious { namespace gpu { namespace device {

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

    cudaMemcpy(_data, cloud.points().data(), cloud.size(), cudaMemcpyHostToDevice);
}

void PointCloud::download(obvious::PointCloud<obvious::PointXyz>& cloud)
{

}

} // end namespace device

} // end namespace gpu

} // end namespace obvious
