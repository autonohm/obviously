#include "obcore/base/Time.h"
#include "obcore/base/Timer.h"

#include <unistd.h>
#include <iostream>

int main(int argc, char** argv)
{
    obvious::Time start(obvious::Time::now());
    obvious::Timer timer;
    timer.start();

    ::sleep(2);
    std::cout << "elapsed time = " << obvious::Time::now() - start << std::endl;
    std::cout << "elapsed time = " << timer.elapsed() << std::endl;
}
