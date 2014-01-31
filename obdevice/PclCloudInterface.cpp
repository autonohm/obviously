#include "PclCloudInterface.h"

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

void PclCloudInterface::save(const double* coords,
                             const unsigned char* rgb,
                             const size_t width,
                             const size_t height,
                             const std::string& fileName)
{
    pcl::PointCloud<pcl::PointXYZRGBL> cloud(width, height);
    const double* coord = coords;
    const unsigned char* colour = rgb;

    for (pcl::PointCloud<pcl::PointXYZRGBL>::iterator point(cloud.begin()); point < cloud.end(); ++point)
    {
        point->x = static_cast<float>(*coord++);
        point->y = static_cast<float>(*coord++);
        point->z = static_cast<float>(*coord++);

        point->r = *colour++;
        point->g = *colour++;
        point->b = *colour++;

        point->label = 0;
    }

    pcl::io::savePCDFileASCII<pcl::PointXYZRGBL>(fileName, cloud);
}

void PclCloudInterface::loadAllCloudsFromDirectory(const std::string& dir,
                                                   std::vector<double*>& coords,
                                                   std::vector<unsigned char*>& rgbs,
                                                   std::vector<size_t>& widths,
                                                   std::vector<size_t>& heights)
{
    std::vector<std::string> fileNames;

    PclCloudInterface::getAllFileNamesFromDirectory(dir, fileNames);

    std::cout << "found " << fileNames.size() << " files in " << dir << "." << std::endl;

    pcl::PointCloud<pcl::PointXYZRGBL> cloud;

    for (std::vector<std::string>::const_iterator file(fileNames.begin()); file < fileNames.end(); ++file)
    {
        pcl::io::loadPCDFile(*file, cloud);

        coords.push_back(new double[3 * cloud.size()]);
        rgbs.push_back(new unsigned char[3 * cloud.size()]);
        widths.push_back(cloud.width);
        heights.push_back(cloud.height);

        double* coord = coords.back();
        unsigned char* colour = rgbs.back();

        for (pcl::PointCloud<pcl::PointXYZRGBL>::const_iterator point(cloud.begin()); point < cloud.end(); ++point)
        {
            *coord++ = point->x;
            *coord++ = point->y;
            *coord++ = point->z;

            *colour++ = point->r;
            *colour++ = point->g;
            *colour++ = point->b;
        }
    }
}

void PclCloudInterface::getAllFileNamesFromDirectory(const std::string& dir, std::vector<std::string>& fileNames)
{
    DIR *dp;
    struct dirent *dirp;

    if ((dp = opendir(dir.c_str())) == NULL)
    {
        std::cout << "Error(" << errno << ") opening " << dir << std::endl;
        return;
    }

    while ((dirp = readdir(dp)) != NULL)
    {
        fileNames.push_back(std::string(dirp->d_name));
    }

    closedir(dp);

    return;
}
