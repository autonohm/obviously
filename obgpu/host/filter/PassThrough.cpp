#include "obgpu/host/filter/PassThrough.h"
#include "obgpu/device/filter/PassThrough.cu"

#include <cuda_runtime_api.h>

namespace obvious { namespace gpu { namespace host { namespace filter {

PassThrough::PassThrough(void)
    : Filter(),
      _xMin(std::numeric_limits<float>::min()),
      _xMax(std::numeric_limits<float>::max()),
      _yMin(_xMin),
      _yMax(_xMax),
      _zMin(_xMin),
      _zMax(_xMax),
      _limits(0)
{
    cudaMalloc(&_limits, 6 * sizeof(float));
}

PassThrough::~PassThrough(void)
{
    cudaFree(_limits);
}

void PassThrough::filter(obvious::gpu::PointCloud* cloud)
{
    cudaMemcpy(_limits, &_xMin, 6 * sizeof(float), cudaMemcpyHostToDevice);

    obvious::gpu::device::filter::filter(_input.points(), cloud, _limits);
}


} // end namespace filter

} // end namespace host

} // end namespace gpu

} // end namespace obvious
