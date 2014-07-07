#include "Time.h"

#include <cmath>

namespace obvious {

Time Time::now(void)
{
    Time time;
    ::gettimeofday(&time._time, 0);
    return time;
}

double Time::operator-(const Time& time) const
{
    return this->sec() - time.sec();
}

Time& Time::operator+=(const double sec)
{
    _time.tv_sec += static_cast<int>(sec);
    _time.tv_usec += std::fmod(1.0f, sec) * 1.0e-6;
    return *this;
}

} // end namespace obvious
