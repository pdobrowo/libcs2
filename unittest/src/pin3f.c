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
#include "cs2/pin3f.h"
#include "unittest/unittest.h"
#include <math.h>

#define EPS (10e-8)
#define test_almost_equal(x, y) TEST_ASSERT_TRUE(fabs((x) - (y)) < EPS)

struct ctx
{
    struct cs2_vec3f_s v123, v357;
};

static void ctx_init(struct ctx *c)
{
    cs2_vec3f_set(&c->v123, 1.0, 2.0, 3.0);
    cs2_vec3f_set(&c->v357, 3.0, 5.0, 7.0);
}

TEST_SUITE(pin3f)

TEST_CASE(pin3f, cl)
{
    struct ctx c;
    struct cs2_pin3f_s p;
    ctx_init(&c);
    cs2_pin3f_cl(&p, &c.v123, &c.v357);
    test_almost_equal(p.p0, 34.0);
    test_almost_equal(p.p12, -1.0);
    test_almost_equal(p.p23, -1.0);
    test_almost_equal(p.p31, 2.0);
}
