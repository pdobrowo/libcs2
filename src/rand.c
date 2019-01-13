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
#include "cs2/rand.h"
#include "cs2/timer.h"
#include <unistd.h>

static uint64_t _cs2_xorshift128plus(struct cs2_rand_s *r)
{
    uint64_t x = r->state[0];
    uint64_t const y = r->state[1];
    r->state[0] = y;
    x ^= x << 23;
    r->state[1] = x ^ y ^ (x >> 17) ^ (y >> 26);
    return r->state[1] + y;
}

void cs2_rand_seed(struct cs2_rand_s *r)
{
    r->state[0] = cs2_timer_nsec();
    r->state[1] = (uint64_t)getpid();
}

double cs2_rand_1f(struct cs2_rand_s *r)
{
    return (double)_cs2_xorshift128plus(r) / (double)UINT64_MAX;
}

double cs2_rand_u1f(struct cs2_rand_s *r, double min, double max)
{
    return min + cs2_rand_1f(r) * (max - min);
}

int cs2_rand_1i(struct cs2_rand_s *r)
{
    return _cs2_xorshift128plus(r) & 1;
}

int cs2_rand_u1i(struct cs2_rand_s *r, int min, int max)
{
    return min + (double)_cs2_xorshift128plus(r) * (max - min) / UINT64_MAX; // fixme: overflow possble
}

void cs2_rand_vec3f_1f(struct cs2_rand_s *r, struct cs2_vec3f_s *v)
{
    cs2_vec3f_set(v, cs2_rand_1f(r), cs2_rand_1f(r), cs2_rand_1f(r));
}

void cs2_rand_vec3f_u1f(struct cs2_rand_s *r, struct cs2_vec3f_s *v, double min, double max)
{
    cs2_vec3f_set(v, cs2_rand_u1f(r, min, max), cs2_rand_u1f(r, min, max), cs2_rand_u1f(r, min, max));
}
