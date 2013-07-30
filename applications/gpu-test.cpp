#include "obcore/base/PointCloud.h"
#include "obdevice/OpenNiDevice.h"
#include "obgpu/PointCloud.h"
#include "obgpu/filter/PassThrough.h"
#include "obgpu/features/NormalEstimator.h"

#include <iostream>
#include <ctime>

using namespace obvious;

namespace {
const int LOOPS = 1000;
}

void grab(OpenNiDevice& sensor, PointCloud<PointXyz>& cloud)
{
    sensor.grab();
    std::vector<float>::const_iterator coord(sensor.coords().begin());

    for (PointCloud<PointXyz>::iterator point(cloud.begin()); point < cloud.end(); ++point)
    {
        point->x = *coord; ++coord;
        point->y = *coord; ++coord;
        point->z = *coord; ++coord;
    }
}

int main(void)
{
    OpenNiDevice sensor;
    PointCloud<PointXyz> cloud;
    gpu::PointCloud gpuCloud;
    gpu::filter::PassThrough filter;
    gpu::features::NormalEstimator estimator;

    sensor.init();
    cloud.resize(sensor.width() * sensor.height());

    std::cout << std::endl << std::endl;
    std::cout << "start " << LOOPS << " loops:" << std::endl;

    for (int i = 0; i < LOOPS; i++)
    {
        grab(sensor, cloud);

        const int clockBegin = clock();

        gpuCloud.upload(cloud);
//        filter.setInput(&gpuCloud);
//        filter.filter(&gpuCloud);
        estimator.setSource(&gpuCloud);
        estimator.estimate();
        gpuCloud.download(cloud);

        std::cout << "loop " << i + 1 << ": " << clock() - clockBegin << " clocks needed." << std::endl;
    }
}
