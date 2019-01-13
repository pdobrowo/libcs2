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
#include "cs2/bezierqq1f.h"
#include "cs2/rand.h"
#include "unittest/unittest.h"
#include <math.h>

#define EPS (10e-8)
#define test_almost_equal(x, y) TEST_ASSERT_TRUE(fabs((x) - (y)) < EPS)

static double fn1(double u, double v)
{
    return 2.0 * u * u + 3.0 * u * v - 5 * v * v;
}

TEST_SUITE(bezierqq1f)

TEST_CASE(bezierqq1f, bump_1)
{
    struct cs2_bezierqq1f_s b;
    struct cs2_bezierqq1f_coeff_s c = { 0, 0, 0,
                                    0, 1, 0,
                                    0, 0, 0 };
    cs2_bezierqq1f_from_qq(&b, &c);

    /* control points */
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.0, 0.0), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.5, 0.0), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 1.0, 0.0), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.0, 0.5), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.5, 0.5), 1.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 1.0, 0.5), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.0, 1.0), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.5, 1.0), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 1.0, 1.0), 0.0);

    /* border */
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.1, 0.0), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.4, 0.0), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.6, 0.0), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.9, 0.0), 0.0);

    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.1, 1.0), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.4, 1.0), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.6, 1.0), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.9, 1.0), 0.0);

    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.0, 0.1), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.0, 0.4), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.0, 0.6), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.0, 0.9), 0.0);

    test_almost_equal(cs2_bezierqq1f_eval(&b, 1.0, 0.1), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 1.0, 0.4), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 1.0, 0.6), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 1.0, 0.9), 0.0);

    /* middle */
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.25, 0.25), 9.0 / 16.0);
}

TEST_CASE(bezierqq1f, bump_2)
{
    struct cs2_bezierqq1f_s b;
    struct cs2_bezierqq1f_coeff_s c = { 0, 1, 0,
                                    1, 1, 1,
                                    0, 1, 0 };
    cs2_bezierqq1f_from_qq(&b, &c);

    /* control points */
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.0, 0.0), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.5, 0.0), 1.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 1.0, 0.0), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.0, 0.5), 1.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.5, 0.5), 1.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 1.0, 0.5), 1.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.0, 1.0), 0.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.5, 1.0), 1.0);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 1.0, 1.0), 0.0);

    /* border */
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.0, 0.25), 0.75);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.0, 0.75), 0.75);

    test_almost_equal(cs2_bezierqq1f_eval(&b, 1.0, 0.25), 0.75);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 1.0, 0.75), 0.75);

    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.25, 0.0), 0.75);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.75, 0.0), 0.75);

    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.25, 1.0), 0.75);
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.75, 1.0), 0.75);

    /* middle */
    test_almost_equal(cs2_bezierqq1f_eval(&b, 0.25, 0.25), 15.0 / 16.0);
}

TEST_CASE(bezierqq1f, func_fn1)
{
    struct cs2_bezierqq1f_s b;
    struct cs2_bezierqq1f_coeff_s c = { fn1(0.0, 0.0), fn1(0.0, 0.5), fn1(0.0, 1.0),
                                    fn1(0.5, 0.0), fn1(0.5, 0.5), fn1(0.5, 1.0),
                                    fn1(1.0, 0.0), fn1(1.0, 0.5), fn1(1.0, 1.0) };
    struct cs2_rand_s r;
    double u, v;
    int i;

    cs2_rand_seed(&r);

    cs2_bezierqq1f_from_qq(&b, &c);

    for (i = 0; i < 1000; ++i)
    {
        u = cs2_rand_u1f(&r, 0.0, 1.0);
        v = cs2_rand_u1f(&r, 0.0, 1.0);

        test_almost_equal(cs2_bezierqq1f_eval(&b, u, v), fn1(u, v));
    }
}
