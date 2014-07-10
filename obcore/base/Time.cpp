#include "Time.h"

#include <cmath>
#include <sys/time.h>

namespace obvious {

Time Time::now(void)
{
    Time time;
    timeval clock;
    ::gettimeofday(&clock, 0);
    time._seconds = clock.tv_sec;
    time._mus = clock.tv_usec;

    return time;
}

double Time::hours(void) const
{
    return static_cast<double>(_seconds) / 3600.0f;
}

double Time::min(void) const
{
    return static_cast<double>(_seconds) / 60.0f;
}

double Time::sec(void) const
{
    return static_cast<double>(_seconds) + static_cast<double>(_mus) * 1.0e-6;
}

double Time::mus(void) const
{
    return static_cast<double>(_seconds) * 1.0e6 + static_cast<double>(_mus);
}

double Time::operator-(const Time& time) const
{
    return this->sec() - time.sec();
}

Time& Time::operator+=(const double sec)
{
    _seconds += static_cast<int>(sec);
    _mus += std::fmod(1.0f, sec) * 1.0e-6;
    return *this;
}

std::ostream& operator<<(std::ostream& out, const Time& time)
{
    out << "[" << time._seconds / 3600 << ":"
        << time._seconds % 3600 << ":"
        << time._seconds % 60 << "]";

    return out;
}

} // end namespace obvious
