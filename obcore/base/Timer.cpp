#include "Timer.h"
#include <iostream>
using namespace std;

/**
 * Initial implementation taken from openvolksbot library
 */

namespace obvious
{

/**
 * windows specific time measurement info
 */
#ifdef WIN32
	static LONG64 g_ticksPerSecond = 0;
#endif

Timer::Timer() {
#ifdef WIN32
	 if(!g_ticksPerSecond) QueryPerformanceFrequency((LARGE_INTEGER*)&g_ticksPerSecond);
#endif
	_startTime = getCurrentTime();
}

Timer::~Timer() {
}

long double Timer::reset() {
	_currentTime = getCurrentTime();
	long double timeDiff = _currentTime - _startTime;
	_startTime = _currentTime;
	return timeDiff;
}

long double Timer::getTime() {
	_currentTime = getCurrentTime();
	return (_currentTime - _startTime);
}

long double Timer::getCurrentTime() {
#ifdef WIN32
	LARGE_INTEGER tick;
	QueryPerformanceCounter(&tick);
	tick.QuadPart = tick.QuadPart * 1000 / g_ticksPerSecond;
	return tick.u.LowPart;
#else
  	static struct timeval tv;
  	gettimeofday(&tv, NULL);
	return (long double)(tv.tv_sec*1000.0 + tv.tv_usec/1000.0);
#endif
}

}
