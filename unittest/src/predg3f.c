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
#include "cs2/predg3f.h"
#include "cs2/rand.h"
#include "cs2/spin3f.h"
#include "cs2/spinquad3f.h"
#include "unittest/unittest.h"
#include <math.h>

/**
 * P = K x L
 * Q = A - B
 * U = K - L
 * V = A x B
 * a = ||P|| ||Q||
 * b = ||U|| ||V||
 * c = c
 *
 * Integer example:
 * K = [-3, 10, 4]
 * L = [-1, -6, -4]
 * A = [6, 2, -5]
 * B = [10, -2, -3]
 *
 * P = [-16, -16, 28]
 * Q = [-4, 4, -2]
 * U = [-2, 16, 8]
 * V = [-16, -32, -32]
 *
 * ||P|| = 36
 * ||Q|| = 6
 * ||U|| = 18
 * ||V|| = 48
 *
 * a = 216
 * b = 864
 */

/* common */
static const struct cs2_predg3f_s test_predg3f_an_empty_set = {
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    0.0
};

/* ellipsoidal */
static const struct cs2_predg3f_s test_predg3f_a_pair_of_points = {
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    { 6.0, 2.0, -5.0 },
    { 10.0, -2.0, -3.0 },
    -1080.0
};

static const struct cs2_predg3f_s test_predg3f_a_pair_of_separate_ellipsoids = {
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    { 6.0, 2.0, -5.0 },
    { 10.0, -2.0, -3.0 },
    -1000.0
};

static const struct cs2_predg3f_s test_predg3f_a_pair_of_y_touching_ellipsoids = {
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    { 6.0, 2.0, -5.0 },
    { 10.0, -2.0, -3.0 },
    -648.0
};

static const struct cs2_predg3f_s test_predg3f_a_pair_of_yz_crossed_ellipsoids = {
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    0.0
};

static const struct cs2_predg3f_s test_predg3f_a_pair_of_z_touching_ellipsoids = {
    { 10.0, -2.0, -3.0 },
    { 6.0, 2.0, -5.0 },
    { -1.0, -6.0, -4.0 },
    { -3.0, 10.0, 4.0 },
    -648.0
};

static const struct cs2_predg3f_s test_predg3f_a_y_barrel = {
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    { 6.0, 2.0, -5.0 },
    { 10.0, -2.0, -3.0 },
    17.0
};

static const struct cs2_predg3f_s test_predg3f_a_z_barrel = {
    { 10.0, -2.0, -3.0 },
    { 6.0, 2.0, -5.0 },
    { -1.0, -6.0, -4.0 },
    { -3.0, 10.0, 4.0 },
    17.0
};

static const struct cs2_predg3f_s test_predg3f_a_notched_y_barrel = {
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    { 6.0, 2.0, -5.0 },
    { 10.0, -2.0, -3.0 },
    648.0
};

static const struct cs2_predg3f_s test_predg3f_a_notched_z_barrel = {
    { 10.0, -2.0, -3.0 },
    { 6.0, 2.0, -5.0 },
    { -1.0, -6.0, -4.0 },
    { -3.0, 10.0, 4.0 },
    648.0
};

static const struct cs2_predg3f_s test_predg3f_a_pair_of_separate_yz_caps = {
    { -3.0, 10.0, 4.0 },
    { -1.0, -6.0, -4.0 },
    { 6.0, 2.0, -5.0 },
    { 10.0, -2.0, -3.0 },
    1080.0
};

/* toroidal */
static const struct cs2_predg3f_s test_predg3f_a_xy_zw_torus = {
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    0.0
};

static const struct cs2_predg3f_s test_predg3f_a_xy_circle = {
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    0.0
};

static const struct cs2_predg3f_s test_predg3f_a_zw_circle = {
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    0.0
};

static const struct cs2_predg3f_s test_predg3f_a_xz_yw_torus = {
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    0.0
};

static const struct cs2_predg3f_s test_predg3f_a_xz_circle = {
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    0.0
};

static const struct cs2_predg3f_s test_predg3f_a_yw_circle = {
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    0.0
};

#define EPS (10e-8)

static int _cs2_almost_zero(double x)
{
    return fabs(x) < EPS;
}

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

#if 0 /* TODO: uncomment when all cases are covered */
TEST_CASE(predg3f, param_random)
{
    struct cs2_predgparam3f_s pp;
    struct cs2_predg3f_s g;
    struct cs2_rand_s r;

    const int ITER = 100000;
    int i;

    cs2_rand_seed(&r);

    for (i = 0; i < ITER; ++i)
    {
        rand_predg3f(&r, &g);
        cs2_predg3f_param(&pp, &g);
    }
}
#endif

TEST_CASE(predg3f, param_bug_too_low_epsilon_for_ellipsoidal_param_check)
{
    struct cs2_predgparam3f_s pp;
    struct cs2_predg3f_s g =
    {
        {
            -4.83573351615323,
            4.591556820667995,
            -4.611256698347384
        },
        {
            0.9161617868399894,
            -2.5622936805116296,
            -9.695889783127331
        },
        {
            -6.917355852861977,
            -6.831885097527042,
            8.6774538645717
        },
        {
            -9.492350856142728,
            7.503800005474261,
            3.9511785552260363
        },
        -9.14216716174187
    };

    cs2_predg3f_param(&pp, &g);
}

TEST_CASE(predg3f, param_an_empty_set)
{
    struct cs2_predgparam3f_s pp;
    cs2_predg3f_param(&pp, &test_predg3f_an_empty_set);

    TEST_ASSERT_TRUE(pp.t == cs2_predgparamtype3f_an_empty_set);
    TEST_ASSERT_STRING_EQUAL(cs2_predgparamtype3f_str(pp.t), "an empty set");
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_dim(pp.t) == -1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_domain_components(pp.t) == 0);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_manifold(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_has_domain_hole(pp.t) == 0);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_connected(pp.t) == 1);
}

TEST_CASE(predg3f, param_a_pair_of_points)
{
    struct cs2_predgparam3f_s pp;
    struct cs2_spinquad3f_s sq;
    struct cs2_spin3f_s sp;
    const struct cs2_predg3f_s *pg = &test_predg3f_a_pair_of_points;
    int c;

    cs2_spinquad3f_from_predg3f(&sq, pg);
    cs2_predg3f_param(&pp, pg);

    TEST_ASSERT_TRUE(pp.t == cs2_predgparamtype3f_a_pair_of_points);
    TEST_ASSERT_STRING_EQUAL(cs2_predgparamtype3f_str(pp.t), "a pair of points");
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_dim(pp.t) == 0);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_domain_components(pp.t) == 2);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_manifold(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_has_domain_hole(pp.t) == 0);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_connected(pp.t) == 0);

    for (c = 0; c < 2; ++c)
    {
        cs2_predgparam3f_eval(&sp, &pp, 0.0, 0.0, c);
        TEST_ASSERT_TRUE(_cs2_almost_zero(cs2_spinquad3f_eval(&sq, &sp)));
    }
}

TEST_CASE(predg3f, param_a_pair_of_separate_ellipsoids)
{
    struct cs2_predgparam3f_s pp;
    struct cs2_spinquad3f_s sq;
    struct cs2_spin3f_s sp;
    const struct cs2_predg3f_s *pg = &test_predg3f_a_pair_of_separate_ellipsoids;
    int c;
    double u, v;

    cs2_spinquad3f_from_predg3f(&sq, pg);
    cs2_predg3f_param(&pp, pg);

    TEST_ASSERT_TRUE(pp.t == cs2_predgparamtype3f_a_pair_of_separate_ellipsoids);
    TEST_ASSERT_STRING_EQUAL(cs2_predgparamtype3f_str(pp.t), "a pair of separate ellipsoids");
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_dim(pp.t) == 2);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_domain_components(pp.t) == 2);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_manifold(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_has_domain_hole(pp.t) == 0);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_connected(pp.t) == 0);

    for (c = 0; c < 2; ++c) for (u = 0.0; u <= 1.0; u += 0.01) for (v = 0.0; v <= 1.0; v += 0.01)
    {
        cs2_predgparam3f_eval(&sp, &pp, u, v, c);
        TEST_ASSERT_TRUE(_cs2_almost_zero(cs2_spinquad3f_eval(&sq, &sp)));
    }
}

TEST_CASE(predg3f, param_a_pair_of_y_touching_ellipsoids)
{
    struct cs2_predgparam3f_s pp;
    struct cs2_spinquad3f_s sq;
    struct cs2_spin3f_s sp;
    const struct cs2_predg3f_s *pg = &test_predg3f_a_pair_of_y_touching_ellipsoids;
    int c;
    double u, v;

    cs2_spinquad3f_from_predg3f(&sq, pg);
    cs2_predg3f_param(&pp, pg);

    TEST_ASSERT_TRUE(pp.t == cs2_predgparamtype3f_a_pair_of_y_touching_ellipsoids);
    TEST_ASSERT_STRING_EQUAL(cs2_predgparamtype3f_str(pp.t), "a pair of y-touching ellipsoids");
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_dim(pp.t) == 2);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_domain_components(pp.t) == 2);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_manifold(pp.t) == 0);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_has_domain_hole(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_connected(pp.t) == 1);

    for (c = 0; c < 2; ++c) for (u = 0.0; u <= 1.0; u += 0.01) for (v = 0.0; v <= 1.0; v += 0.01)
    {
        cs2_predgparam3f_eval(&sp, &pp, u, v, c);
        TEST_ASSERT_TRUE(_cs2_almost_zero(cs2_spinquad3f_eval(&sq, &sp)));
    }

    /* TODO: test if values in domain holes match */
}

TEST_CASE(predg3f, param_a_pair_of_yz_crossed_ellipsoids)
{
    struct cs2_predgparam3f_s pp;
    struct cs2_spinquad3f_s sq;
    struct cs2_spin3f_s sp;
    const struct cs2_predg3f_s *pg = &test_predg3f_a_pair_of_yz_crossed_ellipsoids;
    int c;
    double u, v;

    cs2_spinquad3f_from_predg3f(&sq, pg);
    cs2_predg3f_param(&pp, pg);

    TEST_ASSERT_TRUE(pp.t == cs2_predgparamtype3f_a_pair_of_yz_crossed_ellipsoids);
    TEST_ASSERT_STRING_EQUAL(cs2_predgparamtype3f_str(pp.t), "a pair of yz-crossed ellipsoids");
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_dim(pp.t) == 2);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_domain_components(pp.t) == 2);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_manifold(pp.t) == 0);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_has_domain_hole(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_connected(pp.t) == 1);

    for (c = 0; c < 2; ++c) for (u = 0.0; u <= 1.0; u += 0.01) for (v = 0.0; v <= 1.0; v += 0.01)
    {
        cs2_predgparam3f_eval(&sp, &pp, u, v, c);
        TEST_ASSERT_TRUE(_cs2_almost_zero(cs2_spinquad3f_eval(&sq, &sp)));
    }

    /* TODO: test if values in domain holes match */
}

TEST_CASE(predg3f, param_a_pair_of_z_touching_ellipsoids)
{
    struct cs2_predgparam3f_s pp;
    struct cs2_spinquad3f_s sq;
    struct cs2_spin3f_s sp;
    const struct cs2_predg3f_s *pg = &test_predg3f_a_pair_of_z_touching_ellipsoids;
    int c;
    double u, v;

    cs2_spinquad3f_from_predg3f(&sq, pg);
    cs2_predg3f_param(&pp, pg);

    TEST_ASSERT_TRUE(pp.t == cs2_predgparamtype3f_a_pair_of_z_touching_ellipsoids);
    TEST_ASSERT_STRING_EQUAL(cs2_predgparamtype3f_str(pp.t), "a pair of z-touching ellipsoids");
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_dim(pp.t) == 2);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_domain_components(pp.t) == 2);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_manifold(pp.t) == 0);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_has_domain_hole(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_connected(pp.t) == 1);

    for (c = 0; c < 2; ++c) for (u = 0.0; u <= 1.0; u += 0.01) for (v = 0.0; v <= 1.0; v += 0.01)
    {
        cs2_predgparam3f_eval(&sp, &pp, u, v, c);
        TEST_ASSERT_TRUE(_cs2_almost_zero(cs2_spinquad3f_eval(&sq, &sp)));
    }

    /* TODO: test if values in domain holes match */
}

TEST_CASE(predg3f, param_a_y_barrel)
{
    struct cs2_predgparam3f_s pp;
    struct cs2_spinquad3f_s sq;
    struct cs2_spin3f_s sp;
    const struct cs2_predg3f_s *pg = &test_predg3f_a_y_barrel;
    double u, v;

    cs2_spinquad3f_from_predg3f(&sq, pg);
    cs2_predg3f_param(&pp, pg);

    TEST_ASSERT_TRUE(pp.t == cs2_predgparamtype3f_a_y_barrel);
    TEST_ASSERT_STRING_EQUAL(cs2_predgparamtype3f_str(pp.t), "a y-barrel");
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_dim(pp.t) == 2);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_domain_components(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_manifold(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_has_domain_hole(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_connected(pp.t) == 1);

    for (u = 0.0; u <= 1.0; u += 0.01) for (v = 0.0; v <= 1.0; v += 0.01)
    {
        cs2_predgparam3f_eval(&sp, &pp, u, v, 0);
        TEST_ASSERT_TRUE(_cs2_almost_zero(cs2_spinquad3f_eval(&sq, &sp)));
    }

    /* TODO: test if values in domain holes match */
}

TEST_CASE(predg3f, param_a_z_barrel)
{
    struct cs2_predgparam3f_s pp;
    struct cs2_spinquad3f_s sq;
    struct cs2_spin3f_s sp;
    const struct cs2_predg3f_s *pg = &test_predg3f_a_z_barrel;
    double u, v;

    cs2_spinquad3f_from_predg3f(&sq, pg);
    cs2_predg3f_param(&pp, pg);

    TEST_ASSERT_TRUE(pp.t == cs2_predgparamtype3f_a_z_barrel);
    TEST_ASSERT_STRING_EQUAL(cs2_predgparamtype3f_str(pp.t), "a z-barrel");
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_dim(pp.t) == 2);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_domain_components(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_manifold(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_has_domain_hole(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_connected(pp.t) == 1);

    for (u = 0.0; u <= 1.0; u += 0.01) for (v = 0.0; v <= 1.0; v += 0.01)
    {
        cs2_predgparam3f_eval(&sp, &pp, u, v, 0);
        TEST_ASSERT_TRUE(_cs2_almost_zero(cs2_spinquad3f_eval(&sq, &sp)));
    }

    /* TODO: test if values in domain holes match */
}

TEST_CASE(predg3f, param_a_notched_y_barrel)
{
    struct cs2_predgparam3f_s pp;
    struct cs2_spinquad3f_s sq;
    struct cs2_spin3f_s sp;
    const struct cs2_predg3f_s *pg = &test_predg3f_a_notched_y_barrel;
    double u, v;

    cs2_spinquad3f_from_predg3f(&sq, pg);
    cs2_predg3f_param(&pp, pg);

    TEST_ASSERT_TRUE(pp.t == cs2_predgparamtype3f_a_notched_y_barrel);
    TEST_ASSERT_STRING_EQUAL(cs2_predgparamtype3f_str(pp.t), "a notched y-barrel");
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_dim(pp.t) == 2);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_domain_components(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_manifold(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_has_domain_hole(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_connected(pp.t) == 1);

    for (u = 0.0; u <= 1.0; u += 0.01) for (v = 0.0; v <= 1.0; v += 0.01)
    {
        cs2_predgparam3f_eval(&sp, &pp, u, v, 0);
        TEST_ASSERT_TRUE(_cs2_almost_zero(cs2_spinquad3f_eval(&sq, &sp)));
    }

    /* TODO: test if values in domain holes match */
}

TEST_CASE(predg3f, param_a_notched_z_barrel)
{
    struct cs2_predgparam3f_s pp;
    struct cs2_spinquad3f_s sq;
    struct cs2_spin3f_s sp;
    const struct cs2_predg3f_s *pg = &test_predg3f_a_notched_z_barrel;
    double u, v;

    cs2_spinquad3f_from_predg3f(&sq, pg);
    cs2_predg3f_param(&pp, pg);

    TEST_ASSERT_TRUE(pp.t == cs2_predgparamtype3f_a_notched_z_barrel);
    TEST_ASSERT_STRING_EQUAL(cs2_predgparamtype3f_str(pp.t), "a notched z-barrel");
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_dim(pp.t) == 2);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_domain_components(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_manifold(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_has_domain_hole(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_connected(pp.t) == 1);

    for (u = 0.0; u <= 1.0; u += 0.01) for (v = 0.0; v <= 1.0; v += 0.01)
    {
        cs2_predgparam3f_eval(&sp, &pp, u, v, 0);
        TEST_ASSERT_TRUE(_cs2_almost_zero(cs2_spinquad3f_eval(&sq, &sp)));
    }

    /* TODO: test if values in domain holes match */
}

TEST_CASE(predg3f, param_a_pair_of_separate_yz_caps)
{
    struct cs2_predgparam3f_s pp;
    struct cs2_spinquad3f_s sq;
    struct cs2_spin3f_s sp;
    const struct cs2_predg3f_s *pg = &test_predg3f_a_pair_of_separate_yz_caps;
    int c;
    double u, v;

    cs2_spinquad3f_from_predg3f(&sq, pg);
    cs2_predg3f_param(&pp, pg);

    TEST_ASSERT_TRUE(pp.t == cs2_predgparamtype3f_a_pair_of_separate_yz_caps);
    TEST_ASSERT_STRING_EQUAL(cs2_predgparamtype3f_str(pp.t), "a pair of separate yz-caps");
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_dim(pp.t) == 2);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_domain_components(pp.t) == 2);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_manifold(pp.t) == 1);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_has_domain_hole(pp.t) == 0);
    TEST_ASSERT_TRUE(cs2_predgparamtype3f_is_connected(pp.t) == 0);

    for (c = 0; c < 2; ++c) for (u = 0.0; u <= 1.0; u += 0.01) for (v = 0.0; v <= 1.0; v += 0.01)
    {
        cs2_predgparam3f_eval(&sp, &pp, u, v, c);
        TEST_ASSERT_TRUE(_cs2_almost_zero(cs2_spinquad3f_eval(&sq, &sp)));
    }
}

TEST_CASE(predg3f, param_a_xy_zw_torus)
{
}

TEST_CASE(predg3f, param_a_xy_circle)
{
}

TEST_CASE(predg3f, param_a_zw_circle)
{
}

TEST_CASE(predg3f, param_a_xz_yw_torus)
{
}

TEST_CASE(predg3f, param_a_xz_circle)
{
}

TEST_CASE(predg3f, param_a_yw_circle)
{
}
