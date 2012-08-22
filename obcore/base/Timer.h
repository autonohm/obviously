#ifndef TIMER_H__
#define TIMER_H__

#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#include <time.h>
#include <stdio.h>

/**
 * @namespace obvious
 */
namespace obvious
{

/**
 * @class Timer
 * @brief This utility provides functions for precise timing measurements.
 * @author Stefan May, Christopher Loerken and Dirk Holz
 */
class Timer
{
public:
	/**
	 * Default constructor. Starts directly time measurement after instantiation.
	 */
	Timer();

	/**
	 *  Default destructor
	 */
	~Timer();

	/**
	 * Function resets the timer. '_ldStartTime' is set to the current system time.
	 * @return elapsed time in ms.µs since construction of timer or last reset call
	 */
	long double reset();

	/**
	 * Retrieve the elapsed time (in ms.µs) since 
	 * @return elapsed time in ms.µs since construction of timer or last reset call
	 */
	long double getTime();

private:

	/** 
	 * Holds the system time at construction of timer of last reset()-function call
	 */
	long double _startTime;
	
	/**
	 *  Holds current system time
	 */
	long double _currentTime;
	
	/**
	 * Function retrieves the current time from operating system 
	 * using system specific standard functions. The Current Time
	 * in ms.µs is stored in _ldCurrentTime.
	 * @return elapsed time in ms.µs since construction of timer or last reset call
	 */
	long double getCurrentTime();

	/**
	 * Windows specific time measurement information
	 */
	#ifdef WIN32
		LARGE_INTEGER _ticksPerSecond;
	#endif
};

} /*namespace*/

#endif /*TIMER_H__*/
