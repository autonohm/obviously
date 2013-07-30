#include "obgpu/features/NormalEstimator.h"
#include "obgpu/search/BruteForce.h"
#include "obgpu/point-types.h"

#include <cuda_runtime_api.h>

#include <limits>
#include <iostream>

namespace obvious { namespace gpu { namespace features {

__device__ float machine_float_epsilon(void)
{
    typedef union {
        int i32;
        float f32;
    } flt_32;

    flt_32 s;

    s.f32 = 1.;
    s.i32++;

    return (s.f32 - 1.);
}

struct RefVector3f
{
    __device__ __forceinline__ RefVector3f(float* a, float* b, float* c)
    {
        _data[0] = a;
        _data[1] = b;
        _data[2] = c;
    }

    __device__ __forceinline__ float& operator[](int index) { return *_data[index]; }
    __device__ __forceinline__ const float& operator[](int index) const { return *_data[index]; }
    __device__ __forceinline__ RefVector3f& operator=(const RefVector3f& ref)
    {
        _data[0] = ref._data[0];
        _data[1] = ref._data[1];
        _data[2] = ref._data[2];

        return *this;
    }

private:
    float* _data[3];
};

struct Vector3f
{
    __device__ Vector3f(const float value = 0.0)
    {
        _data[0] = value;
        _data[1] = value;
        _data[2] = value;
    }
    __device__ Vector3f(const Vector3f& vector)
    {
        *this = vector;
    }
    __device__ Vector3f(const float a, const float b, const float c)
    {
        _data[0] = a;
        _data[1] = b;
        _data[2] = c;
    }
    __device__ __forceinline__ float& operator[](int index) { return _data[index]; }
    __device__ __forceinline__ const float& operator[](int index) const { return _data[index]; }
    __device__ __forceinline__ Vector3f& operator=(const Vector3f& vector)
    {
        _data[0] = vector._data[0];
        _data[1] = vector._data[1];
        _data[2] = vector._data[2];

        return *this;
    }
    __device__ __forceinline__ Vector3f operator/(const float scalar) const { return Vector3f(*this) /= scalar; }
    __device__ __forceinline__ Vector3f& operator/=(const float scalar) { return *this *= 1.0 / scalar; }
    __device__ __forceinline__ Vector3f operator*(const float scalar) const { return Vector3f(*this) *= scalar; }
    __device__ __forceinline__ Vector3f& operator*=(const float scalar)
    {
        _data[0] *= scalar;
        _data[1] *= scalar;
        _data[2] *= scalar;

        return *this;
    }
    __device__ __forceinline__ Vector3f operator*(const Vector3f& vector) const
    {
        return Vector3f(_data[0] * vector._data[0],
                        _data[1] * vector._data[1],
                        _data[2] * vector._data[2]);
    }
    __device__ int indexOf(const float value) const
    {
        return _data[0] == value ? 0 : _data[1] == value ? 1 : _data[2] == value ? 2 : -1;
    }
    __device__ Vector3f cross(const Vector3f& vector) const
    {
        return Vector3f(_data[1] * vector._data[2] - _data[2] * vector._data[1],
                        _data[2] * vector._data[0] - _data[0] * vector._data[2],
                        _data[0] * vector._data[1] - _data[1] * vector._data[0]);
    }
    __device__ __forceinline__ float sum(void) const { return _data[0] + _data[1] + _data[2]; }
    __device__ float max(void) const
    {
        float max = _data[0];

        if (_data[1] > max) max = _data[1];
        if (_data[2] > max) max = _data[2];

        return max;
    }
    __device__ __forceinline__ float dot(const Vector3f& vector) const { return (*this * vector).sum(); }
    __device__ __forceinline__ float squaredLength(void) const { return Vector3f (*this * *this).sum(); }
    __device__ Vector3f& sortMax(void)
    {
        if (_data[1] > _data[0])
        {
            this->swap(1, 0);
        }
        if (_data[2] > _data[1])
        {
            this->swap(2, 1);

            if (_data[1] > _data[0])
                this->swap(1, 0);
        }

        return *this;
    }
    __device__ Vector3f& sortMin(void)
    {
        if (_data[1] < _data[0])
        {
            this->swap(1, 0);
        }
        if (_data[2] < _data[1])
        {
            this->swap(2, 1);

            if (_data[1] < _data[0])
                this->swap(1, 0);
        }

        return *this;
    }

private:
    __device__ __forceinline__ void swap(const int a, const int b)
    {
        const float tmp = _data[a];
        _data[a] = _data[b];
        _data[b] = tmp;
    }

    float _data[3];
};

struct Matrix3f {
    __device__ __forceinline__ Matrix3f(const float value = 0.0)
    {
        _data[0] = Vector3f(value);
        _data[1] = Vector3f(value);
        _data[2] = Vector3f(value);
    }
    __device__ __forceinline__ Matrix3f(const Matrix3f& matrix)
    {
        *this = matrix;
    }
    __device__ __forceinline__ Vector3f& operator[](const int index) { return _data[index]; }
    __device__ __forceinline__ const Vector3f& operator[](int index) const { return _data[index]; }
    __device__ __forceinline__ float& operator()(const int row, const int col) { return _data[row][col]; }
    __device__ __forceinline__ const float& operator()(const int row, const int col) const { return _data[row][col]; }
    __device__ __forceinline__ Matrix3f& operator/=(const float scalar)
    {
        _data[0] /= scalar;
        _data[1] /= scalar;
        _data[2] /= scalar;

        return *this;
    }
    __device__ __forceinline__ Vector3f& row(const unsigned int index) { return _data[index]; }
    __device__ __forceinline__ const Vector3f& row(const unsigned int index) const { return _data[index]; }
    __device__ RefVector3f diagonal(void) { return RefVector3f(&_data[0][0], &_data[1][1], &_data[2][2]); }

    __device__ float max(void) const
    {
        const float max0 = _data[0].max();
        const float max1 = _data[1].max();
        const float max2 = _data[2].max();
        float max = max0;

        if (max1 > max) max = max1;
        if (max2 > max) max = max2;

        return max;
    }
    __device__ __forceinline__ Matrix3f& normalize(void) { return *this /= this->max(); }

private:
    Vector3f _data[3];
};

__device__ bool compute_mean_and_covariance(const gpu::PointXyz* source, const size_t n, Matrix3f& covariance, Vector3f& centroid)
{
    const gpu::PointXyz* end = source + n;
    const gpu::PointXyz& point = source[blockIdx.x * blockDim.x + threadIdx.x];
    float accu[9] = { 0.0 };
    unsigned int counter = 0;

    for (const gpu::PointXyz* neighbor = source; source < end; ++neighbor)
    {
        if (neighbor->x == NAN || neighbor->y == NAN || neighbor->z == NAN)
            continue;

        gpu::PointXyz diff(neighbor->x - point.x, neighbor->y - point.y, neighbor->z - point.z);

        if (diff.x * diff.x + diff.y * diff.y + diff.z * diff.z > 0.1)
            continue;

        accu[0] += neighbor->x * neighbor->x;
        accu[1] += neighbor->x * neighbor->y;
        accu[2] += neighbor->x * neighbor->z;
        accu[3] += neighbor->y * neighbor->y;
        accu[4] += neighbor->y * neighbor->z;
        accu[5] += neighbor->z * neighbor->z;
        accu[6] += neighbor->x;
        accu[7] += neighbor->y;
        accu[8] += neighbor->z;
        counter++;
    }

    if (!counter)
        return false;

    for (unsigned int i = 0; i < counter; ++i)
        accu[i] /= static_cast<float>(counter);

    centroid[0] = accu[6];
    centroid[1] = accu[7];
    centroid[2] = accu[8];

    covariance(0, 0) = accu[0] - accu[6] * accu[6];
    covariance(0, 1) = accu[1] - accu[6] * accu[7];
    covariance(0, 2) = accu[2] - accu[6] * accu[8];
    covariance(1, 1) = accu[3] - accu[7] * accu[7];
    covariance(1, 2) = accu[4] - accu[7] * accu[8];
    covariance(2, 2) = accu[5] - accu[8] * accu[8];
    covariance(1, 0) = covariance(0, 1);
    covariance(2, 0) = covariance(0, 2);
    covariance(2, 1) = covariance(1, 2);

    return true;
}

__device__ void compute_roots_quadratic(const float b, const float c, Vector3f& roots)
{
    roots[0] = 0.0;
    float d = b * b - 4.0 * c;

    if (d < 0.0) d = 0.0;

    const float sqrt_d = sqrtf(d);

    roots[1] = 0.5 * (b - sqrt_d);
    roots[2] = 0.5 * (b + sqrt_d);
}

__device__ void compute_roots(const Matrix3f& m, Vector3f& roots)
{
    const float c0 = m(0, 0) * m(1, 1) * m(2, 2)
                   + m(0, 1) * m(0, 2) * m(1, 2) * 2
                   - m(0, 0) * m(1, 2) * m(1, 2)
                   - m(1, 1) * m(0, 2) * m(0, 2)
                   - m(2, 2) * m(0, 1) * m(0, 1);

    const float c1 = m(0, 0) * m(1, 1)
                   - m(0, 1) * m(0, 1)
                   + m(0, 0) * m(2, 2)
                   - m(0, 2) * m(0, 2)
                   + m(1, 1) * m(2, 2)
                   - m(1, 2) * m(1, 2);

    const float c2 = m(0, 0) + m(1, 1) + m(2, 2);

    if (fabsf(c0) < machine_float_epsilon())
    {
        compute_roots_quadratic(c2, c1, roots);
        return;
    }

    const float inv3 = 1.0 / 3.0;
    const float sqrt3 = sqrtf(3.0);
    const float c2_over_3 = c2 * inv3;
    float a_over_3 = (c1 - c2 * c2_over_3) * inv3;

    if (a_over_3 > 0)
        a_over_3 = 0.0;


    float half_b = 0.5 * (c0 + c2_over_3 * (2.0 * c2_over_3 * c2_over_3 - c1));
    float q = half_b * half_b + a_over_3 * a_over_3 * a_over_3;

    if (q > 0)
        q = 0.0;

    float rho = sqrtf(-a_over_3);
    float theta = atan2f(sqrtf(-q), half_b) * inv3;
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);
    roots[0] = c2_over_3 + 2.0 * rho * cos_theta;
    roots[1] = c2_over_3 - rho * (cos_theta + sqrt3 * sin_theta);
    roots[2] = c2_over_3 - rho * (cos_theta - sqrt3 * sin_theta);

    roots.sortMin();

    if (roots[0] <= 0.0);
        compute_roots_quadratic(c2, c1, roots);
}

__device__ void compute_eigen(const Matrix3f& matrix, float& eigenvalue, Vector3f& eigenvector)
{
    Matrix3f normalized(matrix);

    const float max = normalized.max();
    normalized /= max;

    Vector3f eigenvalues;
    compute_roots(normalized, eigenvalues);
    eigenvalue = eigenvalues[0] * max;
    normalized[0][0] -= eigenvalues[0];
    normalized[1][1] -= eigenvalues[0];
    normalized[2][2] -= eigenvalues[0];

    Matrix3f eigenvectors;
    eigenvectors[0] = normalized.row(0).cross(normalized.row(1));
    eigenvectors[1] = normalized.row(0).cross(normalized.row(2));
    eigenvectors[2] = normalized.row(1).cross(normalized.row(2));

    Vector3f length(eigenvectors.row(0).squaredLength(),
                    eigenvectors.row(1).squaredLength(),
                    eigenvectors.row(2).squaredLength());

    const int maxLenIdx = length.indexOf(length.max());
    eigenvector = eigenvectors.row(maxLenIdx) / sqrtf(length[maxLenIdx]);
}

__device__ void solve_plane_parameters(const Matrix3f& covariance, Vector3f& normal, float& curvature)
{
    float eigenvalue;
    compute_eigen(covariance, eigenvalue, normal);

    const float eigen_sum = covariance(0, 0) + covariance(1, 1) + covariance(2, 2);

    if (eigen_sum != 0.0)
        curvature = fabsf(eigenvalue / eigen_sum);
    else
        curvature = 0.0;
}

__global__ void estimate_normals(const gpu::PointXyz* source, const size_t n, gpu::Normal* normals)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;

    if (i >= n)
        return;
    /*
    Vector3f centroid;
    Matrix3f covariance;

    if (!compute_mean_and_covariance(source, n, covariance, centroid))
    {
        normals[i].x = NAN;
        normals[i].y = NAN;
        normals[i].z = NAN;

        return;
    }

    Vector3f normal;
    float curvature;

    solve_plane_parameters(covariance, normal, curvature);

    normals[i].x = normal[0];
    normals[i].y = normal[1];
    normals[i].z = normal[2];
    normals[i].curvature = curvature;
    */
    normals[i].x = 1.0f;
    normals[i].y = 0.0f;
    normals[i].z = 0.0f;
    normals[i].curvature = 0.0f;
}

void NormalEstimator::estimate(gpu::PointCloud& normals)
{
    if (!d_source || !normals.isNormal())
    {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
        std::cout << "source not valid or point cloud is not from type normal." << std::endl;

        return;
    }

    const size_t n = d_source->size();

    if (normals.size() != d_source->size())
    {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
        std::cout << "source and normal cloud has different size." << std::endl;
        return;
    }

    estimate_normals<<<n / 1024 + 1, 1024>>>(reinterpret_cast<gpu::PointXyz*>(d_source->data()), n, reinterpret_cast<gpu::Normal*>(normals.data()));
}

} // end namespace features

} // end namespace gpu

} // end namespace obvious

