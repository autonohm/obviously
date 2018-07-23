#include "CloudFactory.h"

#include <ctime>
#include <iostream>
#include <sstream>
#include <clocale>

namespace obvious {

#define MSG(x) (std::cout << "CloudFactory: " << (x) << std::endl)

void CloudFactory::generateRandomCloud(PointCloud<Point>& cloud, const std::size_t size)
{
    std::srand(std::time(0));
    cloud.resize(size);

    for (PointCloud<Point>::iterator point(cloud.begin()); point < cloud.end(); ++point)
    {
        point->x = static_cast<obfloat>(std::rand()) / static_cast<obfloat>(RAND_MAX);
        point->y = static_cast<obfloat>(std::rand()) / static_cast<obfloat>(RAND_MAX);
        point->z = static_cast<obfloat>(std::rand()) / static_cast<obfloat>(RAND_MAX);
    }
}

void CloudFactory::generateRandomCloud(PointCloud<PointRgb>& cloud, const std::size_t size)
{
    std::srand(std::time(0));
    cloud.resize(size);

    for (PointCloud<PointRgb>::iterator point(cloud.begin()); point < cloud.end(); ++point)
    {
        point->x = static_cast<obfloat>(std::rand()) / static_cast<obfloat>(RAND_MAX);
        point->y = static_cast<obfloat>(std::rand()) / static_cast<obfloat>(RAND_MAX);
        point->z = static_cast<obfloat>(std::rand()) / static_cast<obfloat>(RAND_MAX);

        point->r = std::rand() % 256;
        point->g = std::rand() % 256;
        point->b = std::rand() % 256;
    }
}

bool CloudFactory::loadCloud(PointCloud<Point>& cloud, const std::string& file)
{
    std::setlocale(LC_NUMERIC, "C");
    std::ifstream stream;
    stream.open(file.c_str(), std::ios::in);

    if (!stream.is_open())
    {
        MSG("can't open file. Read point cloud failed.");
        return false;
    }

    /* Remove unneed header lines. */
    if (!dropLines(stream, 2))
    {
         MSG("Read point cloud failed.");
        return false;
    }

    std::vector<std::string> fields;
    readLineAndSplit(stream, fields);

    if (fields.size() < 4)
    {
        MSG("cloud has not enough fields. Read point cloud failed.");
        return false;
    }


    /* Remove unneed header lines. */
    if (!dropLines(stream, 3))
    {
        MSG("Read point cloud failed.");
        return false;
    }


    readLineAndSplit(stream, fields);
    if (fields.size() != 2 || fields[0] != "WIDTH")
    {
        MSG("width is corrupt. Read point cloud failed.");
        return false;
    }
    const unsigned int width = std::atoi(fields[1].c_str());

    readLineAndSplit(stream, fields);
    if (fields.size() != 2 || fields[0] != "HEIGHT")
    {
        MSG("height is corrupt. Read point cloud failed.");
        return false;
    }
    const unsigned int height = std::atoi(fields[1].c_str());


    /* Remove unneed header lines. */
    if (!dropLines(stream, 3))
    {
        MSG("Read point cloud failed.");
        return false;
    }


    cloud.resize(width, height);

    for (PointCloud<Point>::iterator point(cloud.begin()); point < cloud.end(); ++point)
    {
        readLineAndSplit(stream, fields);

        if (fields.size() < 3)
            break;

        point->x = std::atof(fields[0].c_str());
        point->y = std::atof(fields[1].c_str());
        point->z = std::atof(fields[2].c_str());
    }

    return true;
}

bool CloudFactory::loadCloud(PointCloud<PointRgb>& cloud, const std::string& file)
{
    std::setlocale(LC_NUMERIC, "C");
    std::ifstream stream;
    stream.open(file.c_str(), std::ios::in);

    if (!stream.is_open())
    {
        MSG("can't open file. Read point cloud failed.");
        return false;
    }

    /* Remove unneed header lines. */
    if (!dropLines(stream, 2))
    {
         MSG("Read point cloud failed.");
        return false;
    }

    std::vector<std::string> fields;
    readLineAndSplit(stream, fields);

    if (fields.size() < 5)
    {
        MSG("cloud has not enough fields. Read point cloud failed.");
        return false;
    }


    /* Remove unneed header lines. */
    if (!dropLines(stream, 3))
    {
        MSG("Read point cloud failed.");
        return false;
    }


    readLineAndSplit(stream, fields);
    if (fields.size() != 2 || fields[0] != "WIDTH")
    {
        MSG("width is corrupt. Read point cloud failed.");
        return false;
    }
    const unsigned int width = std::atoi(fields[1].c_str());

    readLineAndSplit(stream, fields);
    if (fields.size() != 2 || fields[0] != "HEIGHT")
    {
        MSG("height is corrupt. Read point cloud failed.");
        return false;
    }
    const unsigned int height = std::atoi(fields[1].c_str());


    /* Remove unneed header lines. */
    if (!dropLines(stream, 3))
    {
        MSG("Read point cloud failed.");
        return false;
    }


    cloud.resize(width, height);

    for (PointCloud<PointRgb>::iterator point(cloud.begin()); point < cloud.end(); ++point)
    {
        readLineAndSplit(stream, fields);

        if (fields.size() < 3)
            break;

        point->x = std::atof(fields[0].c_str());
        point->y = std::atof(fields[1].c_str());
        point->z = std::atof(fields[2].c_str());

        const unsigned int rgb = std::atoi(fields[3].c_str());

        point->b = rgb & 0xff;
        point->g = (rgb & 0xff00) >> 8;
        point->r = (rgb & 0xff0000) >> 16;
    }

    return true;
}

void CloudFactory::readLineAndSplit(std::ifstream& stream, std::vector<std::string>& tokens)
{
    std::string line;

    if (!std::getline(stream, line))
    {
        MSG("unexpected end of the file. Will return.");
        return;
    }

    std::string token;
    std::stringstream ss(line);
    tokens.clear();

    while (ss >> token)
        tokens.push_back(token);
}

bool CloudFactory::dropLines(std::ifstream& stream, const unsigned int lines)
{
    std::string line;

    for (unsigned int i = 0; i < lines; i++)
    {
        if (!std::getline(stream, line))
        {
            MSG("unexpected end of the file. Will return with flase.");
            return false;
        }
    }

    return true;
}

//bool CloudFactory::saveCloud(const PointCloud<Point>& cloud, const std::string& file)
//{
//    return true;
//}

} // end namespace obvious
