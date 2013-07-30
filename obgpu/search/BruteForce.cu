#include "obgpu/search/BruteForce.h"

using namespace obvious;

namespace obvious { namespace gpu { namespace search {
/*
__global__ void radius_search(gpu::PointXyz* source, const float square_radius, const gpu::PointXyz point, bool* inlier)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;

    gpu::PointXyz diff = source[i] - point;
    inlier[i] = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z <= square_radius;
}
*/

__global__ void radius_search(gpu::PointXyz* source, const float square_radius, const gpu::PointXyz* point, bool* inlier)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;

    gpu::PointXyz diff;
    diff.x = source[i].x - point->x;
    diff.y = source[i].y - point->y;
    diff.z = source[i].z - point->z;
    inlier[i] = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z <= square_radius;
}
/*
void BruteForce::radiusSearch(const float radius, const gpu::PointXyz point)
{
    if (!d_source)
        return;

    radius_search<<<d_source->size() / 1024, 1024>>>(reinterpret_cast<gpu::PointXyz*>(d_source->data()), radius * radius, point);
}
*/
void BruteForce::radiusSearch(const float radius, const gpu::PointXyz* point)
{
    if (!d_source)
        return;

    radius_search<<<d_source->size() / 1024, 1024>>>(reinterpret_cast<gpu::PointXyz*>(d_source->data()), radius * radius, point, d_inlier);
}

} // end namespace search

} // end namespace gpu

} // end namespace obvious
