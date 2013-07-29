#include "obgpu/filter/PassThrough.h"
#include "obgpu/PointCloud.h"

namespace obvious { namespace gpu { namespace filter {

__global__ void check_if_inside(obvious::gpu::PointXyz* input, bool* inside, float* limits)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
}

void PassThrough::filter(obvious::gpu::PointCloud* cloud)
{
    cudaMemcpy(_limits, &_xMin, 6 * sizeof(float), cudaMemcpyHostToDevice);
    bool* inside;
    cudaMalloc((void**)&inside, cloud->size() * sizeof(bool));

    check_if_inside<<<1, 1>>>(reinterpret_cast<obvious::gpu::PointXyz*>(cloud->data()), inside, _limits);

    cudaFree(inside);
}

} // end namespace filter

} // end namespace gpu

} // end namespace obvious
