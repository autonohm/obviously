#include "Timer.h"

namespace obvious {

void Timer::start(void)
{
    _start = Time::now();
}

void Timer::reset(void)
{
    _start = Time::now();
}

double Timer::elapsed(void) const
{
    return Time::now() - _start;
}

double Timer::getTime(void)
{
    const double elapsed = this->elapsed();
    this->reset();
    return elapsed;
}

} // end namespace obvious
