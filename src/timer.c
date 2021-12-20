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
#include "cs2/arch.h"
#include "cs2/assert.h"
#include <time.h>
#if defined(CS2_ARCH_MSYS)
#include <windows.h>
#endif /* defined(CS2_ARCH_MSYS) */

#if defined(CS2_ARCH_LINUX) || defined(CS2_ARCH_BSD) || defined(CS2_ARCH_SOLARIS) || defined(CS2_ARCH_SUNOS)

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

#endif /* defined(CS2_ARCH_LINUX) || defined(CS2_ARCH_BSD) || defined(CS2_ARCH_SOLARIS) || defined(CS2_ARCH_SUNOS) */

#if defined(CS2_ARCH_MSYS)

LARGE_INTEGER _cs2_timer_freq = { 0 };

void _cs_timer_init(void)
{
  if (_cs2_timer_freq.QuadPart > 0)
      return;

  if (!QueryPerformanceFrequency(&_cs2_timer_freq))
      CS2_PANIC_MSG("failed to get performance counter frequency");
}

uint64_t cs2_timer_sec(void)
{
    LARGE_INTEGER counter = { 0 };
    _cs_timer_init();

    if (!QueryPerformanceCounter(&counter))
        CS2_PANIC_MSG("failed to get performance counter value");

    return (uint64_t)(counter.QuadPart / _cs2_timer_freq .QuadPart);
}

uint64_t cs2_timer_msec(void)
{
    LARGE_INTEGER counter = { 0 };
    _cs_timer_init();

    if (!QueryPerformanceCounter(&counter))
        CS2_PANIC_MSG("failed to get performance counter value");

    return (uint64_t)(counter.QuadPart * 1000.0 / _cs2_timer_freq .QuadPart);
}

uint64_t cs2_timer_usec(void)
{
    LARGE_INTEGER counter = { 0 };
    _cs_timer_init();

    if (!QueryPerformanceCounter(&counter))
        CS2_PANIC_MSG("failed to get performance counter value");

    return (uint64_t)(counter.QuadPart * 1000000.0 / _cs2_timer_freq .QuadPart);
}

uint64_t cs2_timer_nsec(void)
{
    LARGE_INTEGER counter = { 0 };
    _cs_timer_init();

    if (!QueryPerformanceCounter(&counter))
        CS2_PANIC_MSG("failed to get performance counter value");

    return (uint64_t)(counter.QuadPart * 1000000000.0 / _cs2_timer_freq .QuadPart);
}

#endif /* defined(CS2_ARCH_MSYS) */
