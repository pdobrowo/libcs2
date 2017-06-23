/**
 * Copyright (c) 2015-2017 Przemysław Dobrowolski
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
#include "cs2/predg3f.h"
#include "cs2/rand.h"
#include "unittest/unittest.h"
#include <math.h>

static void rand_predg3f(struct cs2_rand_s *r, struct cs2_predg3f_s *g)
{
    const double MIN = -10;
    const double MAX = 10;

    cs2_rand_vec3f_u1f(r, &g->a, MIN, MAX);
    cs2_rand_vec3f_u1f(r, &g->b, MIN, MAX);
    cs2_rand_vec3f_u1f(r, &g->k, MIN, MAX);
    cs2_rand_vec3f_u1f(r, &g->l, MIN, MAX);
    g->c = cs2_rand_u1f(r, MIN, MAX);
}

TEST_SUITE(predg3f)

TEST_CASE(predg3f, random_param)
{
    struct cs2_predgparam3f_s pp;
    struct cs2_predg3f_s g;
    struct cs2_rand_s r;

    const int ITER = 1000000;
    int i;

    cs2_rand_seed(&r);

    for (i = 0; i < ITER; ++i)
    {
        rand_predg3f(&r, &g);
        cs2_predg3f_param(&pp, &g);
    }
}
