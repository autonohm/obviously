#include "CloudFactory.h"

#include <ctime>

namespace obvious {

void CloudFactory::generateRandomCloud(PointCloud<Point>& cloud, const std::size_t size)
{
    std::srand(std::time(0));
    cloud.resize(size);

    for (PointCloud<Point>::iterator point(cloud.begin()); point < cloud.end(); ++point)
    {
        point->x = static_cast<obfloat>(std::rand()) / RAND_MAX;
        point->y = static_cast<obfloat>(std::rand()) / RAND_MAX;
        point->z = static_cast<obfloat>(std::rand()) / RAND_MAX;
    }
}

} // end namespace obvious
