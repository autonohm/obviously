#ifndef __PCL_CLOUD_INTERFACE_H__
#define __PCL_CLOUD_INTERFACE_H__

#include <string>
#include <vector>

class PclCloudInterface
{
public:
    static void save(const double* coords,
                     const unsigned char* rgb,
                     const size_t width,
                     const size_t height,
                     const std::string& fileName);

    static void loadAllCloudsFromDirectory(const std::string& dir,
                                           std::vector<double*>& coords,
                                           std::vector<unsigned char*>& rgbs,
                                           std::vector<size_t>& widths,
                                           std::vector<size_t>& heights);

private:
    static void getAllFileNamesFromDirectory(const std::string& dir, std::vector<std::string>& fileNames);
};

#endif
