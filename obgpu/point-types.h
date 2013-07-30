#ifndef __GPU_POINT_TYPES__
#define __GPU_POINT_TYPES__

#include <cstddef>

#include <cuda_runtime_api.h>

namespace obvious { namespace gpu {

struct PointXyz
{
    __device__ PointXyz(void) { }
    __device__ PointXyz(const float valX, const float valY, const float valZ) : x(valX), y(valY), z(valZ) { }
    __device__ PointXyz(const PointXyz& point) : x(point.x), y(point.y), z(point.z) { }

    __device__ PointXyz& operator=(const PointXyz& point)
    {
        x = point.x;
        y = point.y;
        z = point.z;

        return *this;
    }

    float x;
    float y;
    float z;
};

struct Normal : public PointXyz
{
    __device__ Normal(void) : PointXyz() { }
    __device__ Normal(const float valX, const float valY, const float valZ, const float valCurvature)
        : PointXyz(valX, valY, valZ), curvature(valCurvature) { }
    __device__ Normal(const Normal& normal) : PointXyz(normal.x, normal.y, normal.z), curvature(normal.curvature) { }

    __device__ Normal& operator=(const Normal& normal)
    {
        *this = normal;
        curvature = normal.curvature;

        return *this;
    }

    float curvature;
};

} // end namespace gpu

} // end namespace obvious

#endif
