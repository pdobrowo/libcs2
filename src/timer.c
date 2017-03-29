/**
 * Copyright (c) 2015-2019 Przemysław Dobrowolski
 *
 * This file is part of the Configuration Space Library (libcs2), a library
 * for creating configuration spaces of various motion planning problems.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "cs2/timer.h"

#if defined(__linux__) || defined(__FreeBSD__) || defined(__OpenBSD__) || defined(__NetBSD__) || defined(__DragonFly__)

#include <time.h>

uint64_t cs2_timer_sec(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec;
}

uint64_t cs2_timer_msec(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

uint64_t cs2_timer_usec(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + (uint64_t)ts.tv_nsec / 1000;
}

uint64_t cs2_timer_nsec(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000000 + (uint64_t)ts.tv_nsec;
}

#endif /* defined(__linux__) || defined(__FreeBSD__) || defined(__OpenBSD__) || defined(__NetBSD__) || defined(__DragonFly__) */

#if defined(__MINGW32__)

#include "cs2/assert.h"

#define WIN32_LEAN_AND_MEAN
#define VC_EXTRALEAN

#include <windows.h>

LONGLONG _cs2_timer_query_freq()
{
    LARGE_INTEGER li;

    if (!QueryPerformanceFrequency(&li))
        CS2_PANIC_MSG("QueryPerformanceFrequency() failed");

    return li.QuadPart;
}

LONGLONG _cs2_timer_freq = 0;

LONGLONG _cs2_timer_query_time_nsec()
{
    LARGE_INTEGER li;

	if (!_cs2_timer_freq)
		_cs2_timer_freq = _cs2_timer_query_freq();

    if (!QueryPerformanceCounter(&li))
        CS2_PANIC_MSG("QueryPerformanceCounter() failed");

    return 1000000000 * li.QuadPart / _cs2_timer_freq;
}

uint64_t cs2_timer_sec(void)
{
    return (uint64_t)(_cs2_timer_query_time_nsec() / 1000000000);
}

uint64_t cs2_timer_msec(void)
{
    return (uint64_t)(_cs2_timer_query_time_nsec() / 1000000);
}

uint64_t cs2_timer_usec(void)
{
    return (uint64_t)(_cs2_timer_query_time_nsec() / 1000);
}

uint64_t cs2_timer_nsec(void)
{
    return (uint64_t)_cs2_timer_query_time_nsec();
}

#endif /* defined(__MINGW32__) */
