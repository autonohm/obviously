#include "obgpu/filter/PassThrough.h"

#include <cuda_runtime_api.h>

namespace obvious { namespace gpu { namespace filter {

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
    cudaMalloc(reinterpret_cast<void**>(&_limits), 6 * sizeof(float));
}

PassThrough::~PassThrough(void)
{
    cudaFree(_limits);
}


} // end namespace filter

} // end namespace gpu

} // end namespace obvious
