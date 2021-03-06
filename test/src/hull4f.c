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
#include "cs2/hull4f.h"
#include "test/test.h"
#include <math.h>

#define EPS (10e-8)
#define test_almost_equal(x, y) TEST_ASSERT_TRUE(fabs((x) - (y)) < EPS)

static const struct cs2_vec4f_s SIMPLEX_A[] = {
    { 0.0, 0.0, 0.0, 0.0 },
    { 1.0, 0.0, 0.0, 0.0 },
    { 0.0, 1.0, 0.0, 0.0 },
    { 0.0, 0.0, 1.0, 0.0 },
    { 0.0, 0.0, 0.0, 1.0 }
};

static const size_t SIMPLEX_A_SIZE = sizeof(SIMPLEX_A) / sizeof(SIMPLEX_A[0]);

static const struct cs2_vec4f_s SIMPLEX_B[] = {
    { 0.5, 0.0, 0.0, 0.0 },
    { 1.5, 0.0, 0.0, 0.0 },
    { 0.5, 1.0, 0.0, 0.0 },
    { 0.5, 0.0, 1.0, 0.0 },
    { 0.5, 0.0, 0.0, 1.0 }
};

static const size_t SIMPLEX_B_SIZE = sizeof(SIMPLEX_B) / sizeof(SIMPLEX_B[0]);

static const struct cs2_vec4f_s SIMPLEX_C[] = {
    { 2.0, 0.0, 0.0, 0.0 },
    { 3.0, 0.0, 0.0, 0.0 },
    { 2.0, 1.0, 0.0, 0.0 },
    { 2.0, 0.0, 1.0, 0.0 },
    { 2.0, 0.0, 0.0, 1.0 }
};

static const size_t SIMPLEX_C_SIZE = sizeof(SIMPLEX_C) / sizeof(SIMPLEX_C[0]);

static const struct cs2_vec4f_s CUBE_A[] = {
    { 0.0, 0.0, 0.0, 0.0 },
    { 1.0, 0.0, 0.0, 0.0 },
    { 0.0, 1.0, 0.0, 0.0 },
    { 1.0, 1.0, 0.0, 0.0 },
    { 0.0, 0.0, 1.0, 0.0 },
    { 1.0, 0.0, 1.0, 0.0 },
    { 0.0, 1.0, 1.0, 0.0 },
    { 1.0, 1.0, 1.0, 0.0 },
    { 0.0, 0.0, 0.0, 1.0 },
    { 1.0, 0.0, 0.0, 1.0 },
    { 0.0, 1.0, 0.0, 1.0 },
    { 1.0, 1.0, 0.0, 1.0 },
    { 0.0, 0.0, 1.0, 1.0 },
    { 1.0, 0.0, 1.0, 1.0 },
    { 0.0, 1.0, 1.0, 1.0 },
    { 1.0, 1.0, 1.0, 1.0 }
};

static const size_t CUBE_A_SIZE = sizeof(CUBE_A) / sizeof(CUBE_A[0]);

static const struct cs2_vec4f_s CUBE_B[] = {
    { 0.5, 0.0, 0.0, 0.0 },
    { 1.5, 0.0, 0.0, 0.0 },
    { 0.5, 1.0, 0.0, 0.0 },
    { 1.5, 1.0, 0.0, 0.0 },
    { 0.5, 0.0, 1.0, 0.0 },
    { 1.5, 0.0, 1.0, 0.0 },
    { 0.5, 1.0, 1.0, 0.0 },
    { 1.5, 1.0, 1.0, 0.0 },
    { 0.5, 0.0, 0.0, 1.0 },
    { 1.5, 0.0, 0.0, 1.0 },
    { 0.5, 1.0, 0.0, 1.0 },
    { 1.5, 1.0, 0.0, 1.0 },
    { 0.5, 0.0, 1.0, 1.0 },
    { 1.5, 0.0, 1.0, 1.0 },
    { 0.5, 1.0, 1.0, 1.0 },
    { 1.5, 1.0, 1.0, 1.0 }
};

static const size_t CUBE_B_SIZE = sizeof(CUBE_B) / sizeof(CUBE_B[0]);

static const struct cs2_vec4f_s CUBE_C[] = {
    { 2.0, 0.0, 0.0, 0.0 },
    { 3.0, 0.0, 0.0, 0.0 },
    { 2.0, 1.0, 0.0, 0.0 },
    { 3.0, 1.0, 0.0, 0.0 },
    { 2.0, 0.0, 1.0, 0.0 },
    { 3.0, 0.0, 1.0, 0.0 },
    { 2.0, 1.0, 1.0, 0.0 },
    { 3.0, 1.0, 1.0, 0.0 },
    { 2.0, 0.0, 0.0, 1.0 },
    { 3.0, 0.0, 0.0, 1.0 },
    { 2.0, 1.0, 0.0, 1.0 },
    { 3.0, 1.0, 0.0, 1.0 },
    { 2.0, 0.0, 1.0, 1.0 },
    { 3.0, 0.0, 1.0, 1.0 },
    { 2.0, 1.0, 1.0, 1.0 },
    { 3.0, 1.0, 1.0, 1.0 }
};

static const size_t CUBE_C_SIZE = sizeof(CUBE_C) / sizeof(CUBE_C[0]);

TEST_SUITE(hull4f)

TEST_CASE(hull4f, vol_simplex_a)
{
    struct cs2_hull4f_s hsa;

    cs2_hull4f_init(&hsa);
    cs2_hull4f_from_arr(&hsa, SIMPLEX_A, SIMPLEX_A_SIZE);

    test_almost_equal(hsa.vol, 1.0 / 24.0);

    cs2_hull4f_clear(&hsa);
}

TEST_CASE(hull4f, area_simplex_a)
{
    struct cs2_hull4f_s hsa;

    cs2_hull4f_init(&hsa);
    cs2_hull4f_from_arr(&hsa, SIMPLEX_A, SIMPLEX_A_SIZE);

    test_almost_equal(hsa.area, 1.0);

    cs2_hull4f_clear(&hsa);
}

TEST_CASE(hull4f, vol_cube_a)
{
    struct cs2_hull4f_s hca;

    cs2_hull4f_init(&hca);
    cs2_hull4f_from_arr(&hca, CUBE_A, CUBE_A_SIZE);

    test_almost_equal(hca.vol, 1.0);

    cs2_hull4f_clear(&hca);
}

TEST_CASE(hull4f, area_cube_a)
{
    struct cs2_hull4f_s hca;

    cs2_hull4f_init(&hca);
    cs2_hull4f_from_arr(&hca, CUBE_A, CUBE_A_SIZE);

    test_almost_equal(hca.area, 8.0);

    cs2_hull4f_clear(&hca);
}

TEST_CASE(hull4f, sep_cube_abc)
{
    struct cs2_hull4f_s hca, hcb, hcc;

    cs2_hull4f_init(&hca);
    cs2_hull4f_init(&hcb);
    cs2_hull4f_init(&hcc);

    cs2_hull4f_from_arr(&hca, CUBE_A, CUBE_A_SIZE);
    cs2_hull4f_from_arr(&hcb, CUBE_B, CUBE_B_SIZE);
    cs2_hull4f_from_arr(&hcc, CUBE_C, CUBE_C_SIZE);

    TEST_ASSERT_TRUE(cs2_hull4f_inter(&hca, &hcb));
    TEST_ASSERT_TRUE(!cs2_hull4f_inter(&hca, &hcc));
    TEST_ASSERT_TRUE(!cs2_hull4f_inter(&hcb, &hcc));

    cs2_hull4f_clear(&hca);
    cs2_hull4f_clear(&hcb);
    cs2_hull4f_clear(&hcc);
}

TEST_CASE(hull4f, sep_simplex_abc)
{
    struct cs2_hull4f_s hsa, hsb, hsc;

    cs2_hull4f_init(&hsa);
    cs2_hull4f_init(&hsb);
    cs2_hull4f_init(&hsc);

    cs2_hull4f_from_arr(&hsa, SIMPLEX_A, SIMPLEX_A_SIZE);
    cs2_hull4f_from_arr(&hsb, SIMPLEX_B, SIMPLEX_B_SIZE);
    cs2_hull4f_from_arr(&hsc, SIMPLEX_C, SIMPLEX_C_SIZE);

    TEST_ASSERT_TRUE(cs2_hull4f_inter(&hsa, &hsb));
    TEST_ASSERT_TRUE(!cs2_hull4f_inter(&hsa, &hsc));
    TEST_ASSERT_TRUE(!cs2_hull4f_inter(&hsb, &hsc));

    cs2_hull4f_clear(&hsa);
    cs2_hull4f_clear(&hsb);
    cs2_hull4f_clear(&hsc);
}
