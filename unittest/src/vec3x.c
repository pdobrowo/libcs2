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
#include "cs2/vec3x.h"
#include "unittest/unittest.h"
#include <math.h>

struct ctx
{
    struct cs2_vec3x_s v, v123, v357, v248, v159, v789;
    mpz_t si[10];
};

static void ctx_init(struct ctx *c)
{
    int i;
    cs2_vec3x_init(&c->v);
    cs2_vec3x_init(&c->v123);
    cs2_vec3x_init(&c->v357);
    cs2_vec3x_init(&c->v248);
    cs2_vec3x_init(&c->v159);
    cs2_vec3x_init(&c->v789);
    for (i = 0; i <= 9; ++i)
        mpz_init_set_si(c->si[i], i);
    cs2_vec3x_set(&c->v123, c->si[1], c->si[2], c->si[3]);
    cs2_vec3x_set(&c->v357, c->si[3], c->si[5], c->si[7]);
    cs2_vec3x_set(&c->v248, c->si[2], c->si[4], c->si[8]);
    cs2_vec3x_set(&c->v159, c->si[1], c->si[5], c->si[9]);
    cs2_vec3x_set(&c->v789, c->si[7], c->si[8], c->si[9]);
}

static void ctx_clear(struct ctx *c)
{
    int i;
    for (i = 9; i >= 0; --i)
        mpz_clear(c->si[i]);
    cs2_vec3x_clear(&c->v789);
    cs2_vec3x_clear(&c->v159);
    cs2_vec3x_clear(&c->v248);
    cs2_vec3x_clear(&c->v357);
    cs2_vec3x_clear(&c->v123);
    cs2_vec3x_clear(&c->v);
}

TEST_SUITE(vec3x)

TEST_CASE(vec3x, set)
{
    struct ctx c;
    ctx_init(&c);
    cs2_vec3x_set(&c.v, c.si[3], c.si[5], c.si[7]);
    TEST_ASSERT(mpz_cmp_si(c.v.x, 3) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.y, 5) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.z, 7) == 0);
    ctx_clear(&c);
}

TEST_CASE(vec3x, set_si)
{
    struct ctx c;
    ctx_init(&c);
    cs2_vec3x_set_si(&c.v, 3, 5, 7);
    TEST_ASSERT(mpz_cmp_si(c.v.x, 3) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.y, 5) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.z, 7) == 0);
    ctx_clear(&c);
}

TEST_CASE(vec3x, zero)
{
    struct ctx c;
    ctx_init(&c);
    cs2_vec3x_zero(&c.v);
    TEST_ASSERT(mpz_cmp_si(c.v.x, 0) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.y, 0) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.z, 0) == 0);
    ctx_clear(&c);
}

TEST_CASE(vec3x, is_zero)
{
    struct ctx c;
    ctx_init(&c);
    cs2_vec3x_zero(&c.v);
    TEST_ASSERT(cs2_vec3x_is_zero(&c.v));
    ctx_clear(&c);
}

TEST_CASE(vec3x, copy)
{
    struct ctx c;
    ctx_init(&c);
    cs2_vec3x_copy(&c.v, &c.v357);
    TEST_ASSERT(mpz_cmp_si(c.v.x, 3) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.y, 5) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.z, 7) == 0);
    ctx_clear(&c);
}

TEST_CASE(vec3x, add)
{
    struct ctx c;
    ctx_init(&c);
    cs2_vec3x_add(&c.v, &c.v123, &c.v357);
    TEST_ASSERT(mpz_cmp_si(c.v.x, 4) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.y, 7) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.z, 10) == 0);
    ctx_clear(&c);
}

TEST_CASE(vec3x, sub)
{
    struct ctx c;
    ctx_init(&c);
    cs2_vec3x_sub(&c.v, &c.v123, &c.v357);
    TEST_ASSERT(mpz_cmp_si(c.v.x, -2) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.y, -3) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.z, -4) == 0);
    ctx_clear(&c);
}

TEST_CASE(vec3x, neg)
{
    struct ctx c;
    ctx_init(&c);
    cs2_vec3x_neg(&c.v, &c.v123);
    TEST_ASSERT(mpz_cmp_si(c.v.x, -1) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.y, -2) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.z, -3) == 0);
    ctx_clear(&c);
}

TEST_CASE(vec3x, mul)
{
    struct ctx c;
    ctx_init(&c);
    cs2_vec3x_mul(&c.v, &c.v123, c.si[5]);
    TEST_ASSERT(mpz_cmp_si(c.v.x, 5) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.y, 10) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.z, 15) == 0);
    ctx_clear(&c);
}

TEST_CASE(vec3x, cl)
{
    struct ctx c;
    struct cs2_pin3x_s p;
    ctx_init(&c);
    cs2_pin3x_init(&p);
    cs2_vec3x_cl(&p, &c.v123, &c.v357);
    TEST_ASSERT(mpz_cmp_si(p.p0, 34) == 0);
    TEST_ASSERT(mpz_cmp_si(p.p12, -1) == 0);
    TEST_ASSERT(mpz_cmp_si(p.p23, -1) == 0);
    TEST_ASSERT(mpz_cmp_si(p.p31, 2) == 0);
    cs2_pin3x_clear(&p);
    ctx_clear(&c);
}

TEST_CASE(vec3x, mad2)
{
    struct ctx c;
    ctx_init(&c);
    cs2_vec3x_mad2(&c.v, &c.v123, c.si[4], &c.v357, c.si[8]);
    TEST_ASSERT(mpz_cmp_si(c.v.x, 28) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.y, 48) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.z, 68) == 0);
    ctx_clear(&c);
}

TEST_CASE(vec3x, mad3)
{
    struct ctx c;
    ctx_init(&c);
    cs2_vec3x_mad3(&c.v, &c.v123, c.si[4], &c.v357, c.si[8], &c.v248, c.si[9]);
    TEST_ASSERT(mpz_cmp_si(c.v.x, 46) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.y, 84) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.z, 140) == 0);
    ctx_clear(&c);
}

TEST_CASE(vec3x, mad4)
{
    struct ctx c;
    ctx_init(&c);
    cs2_vec3x_mad4(&c.v, &c.v123, c.si[4], &c.v357, c.si[8], &c.v248, c.si[9], &c.v159, c.si[2]);
    TEST_ASSERT(mpz_cmp_si(c.v.x, 48) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.y, 94) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.z, 158) == 0);
    ctx_clear(&c);
}

TEST_CASE(vec3x, mad5)
{
    struct ctx c;
    ctx_init(&c);
    cs2_vec3x_mad5(&c.v, &c.v123, c.si[4], &c.v357, c.si[8], &c.v248, c.si[9], &c.v159, c.si[2], &c.v789, c.si[5]);
    TEST_ASSERT(mpz_cmp_si(c.v.x, 83) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.y, 134) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.z, 203) == 0);
    ctx_clear(&c);
}

TEST_CASE(vec3x, dot)
{
    struct ctx c;
    mpz_t d;
    ctx_init(&c);
    mpz_init(d);
    cs2_vec3x_dot(d, &c.v123, &c.v357);
    TEST_ASSERT(mpz_cmp_si(d, 34) == 0);
    mpz_clear(d);
    ctx_clear(&c);
}

TEST_CASE(vec3x, cross)
{
    struct ctx c;
    ctx_init(&c);
    cs2_vec3x_cross(&c.v, &c.v123, &c.v357);
    TEST_ASSERT(mpz_cmp_si(c.v.x, -1) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.y, 2) == 0);
    TEST_ASSERT(mpz_cmp_si(c.v.z, -1) == 0);
    ctx_clear(&c);
}

TEST_CASE(vec3x, cl_vs_cross_and_dot)
{
    struct ctx c;
    struct cs2_pin3x_s p;
    mpz_t d;
    ctx_init(&c);
    cs2_pin3x_init(&p);
    mpz_init(d);
    cs2_vec3x_cl(&p, &c.v123, &c.v357);
    cs2_vec3x_cross(&c.v, &c.v123, &c.v357);
    cs2_vec3x_dot(d, &c.v123, &c.v357);
    TEST_ASSERT(mpz_cmp(p.p0, d) == 0);
    TEST_ASSERT(mpz_cmp(p.p12, c.v.z) == 0);
    TEST_ASSERT(mpz_cmp(p.p23, c.v.x) == 0);
    TEST_ASSERT(mpz_cmp(p.p31, c.v.y) == 0);
    mpz_clear(d);
    cs2_pin3x_clear(&p);
    ctx_clear(&c);
}

TEST_CASE(vec3x, sqlen)
{
    struct ctx c;
    mpz_t d;
    ctx_init(&c);
    mpz_init(d);
    cs2_vec3x_sqlen(d, &c.v123);
    TEST_ASSERT(mpz_cmp_si(d, 14) == 0);
    mpz_clear(d);
    ctx_clear(&c);
}

TEST_CASE(vec3x, tr)
{
    struct ctx c;
    mpz_t d;
    ctx_init(&c);
    mpz_init(d);
    cs2_vec3x_tr(d, &c.v123);
    TEST_ASSERT(mpz_cmp_si(d, 6) == 0);
    mpz_clear(d);
    ctx_clear(&c);
}
