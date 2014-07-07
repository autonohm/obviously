#include "obcore/base/Time.h"
#include "obcore/base/Timer.h"

#include <unistd.h>
#include <iostream>

#include <Eigen/Core>

namespace {
const unsigned int DIM = 1000000;
}

int main(int argc, char** argv)
{
    obvious::Timer timer;
    timer.start();

    {
        Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> mat(DIM, 3);
    }

    std::cout << "elapsed time = " << timer.elapsed() << std::endl;
}
