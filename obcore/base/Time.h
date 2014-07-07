/*
 * Software License Agreement (BSD License)
 *
 *  Open Robotic Vision and Utilities Library
 *  Copyright (c) 2014 TH-Nuernberg - Christian Merkl
 *  E-Mail: christian.merkl@th-nuernberg.de
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __TIME_H__
#define __TIME_H__

#include <sys/time.h>

namespace obvious {

/**
 * @class Time
 * @brief A simple class to get the current time in sec and usec from the date 1970. You can easly calculate a
 * duration time with the operator-.
 * @author Christian Merkl
 */
class Time
{
public:
    /**
     * Default constructor. Sets all members to 0.
     */
    Time(void) { _time.tv_sec = 0; _time.tv_usec = 0; }
    /**
     * Copy constructor
     * @param time will be copied.
     */
    Time(const Time& time) : _time(time._time) { }

    /**
     * Gets the time in sec.
     * @return sec since 1970.
     */
    double sec(void) const { return _time.tv_sec + _time.tv_usec * 1.0e-6; }
    /**
     * Gets the time in usec.
     * @return usec since 1970.
     */
    double usec(void) const { return _time.tv_sec * 1.0e6 + _time.tv_usec; }
    /**
     * Gets the current time.
     * @return the current time as Time object.
     */
    static Time now(void);
    /**
     * Assignment operator.
     * @param time will be copied.
     */
    Time& operator=(const Time& time)
    {
        _time = time._time;
        return *this;
    }
    /**
     * Calculates the time difference between this and time object.
     * @param time will be subtracted from this.
     */
    double operator-(const Time& time) const;
    /**
     * Adds the given sec to this object.
     * @param sec will be added to this object.
     */
    Time& operator+=(const double sec);

private:
    timeval _time;
};

} // end namespace obvious

#endif
