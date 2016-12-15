/**
 * Copyright (c) 2015-2016 Przemysław Dobrowolski
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
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "cs2/vec3f.h"
#include <criterion/criterion.h>
#include <math.h>

struct ctx
{
    struct vec3f_s v, v123, v357, v248, v159, v789;
};

static void ctx_init(struct ctx *c)
{
    vec3f_set(&c->v123, 1.0, 2.0, 3.0);
    vec3f_set(&c->v357, 3.0, 5.0, 7.0);
    vec3f_set(&c->v248, 2.0, 4.0, 8.0);
    vec3f_set(&c->v159, 1.0, 5.0, 9.0);
    vec3f_set(&c->v789, 7.0, 8.0, 9.0);
}

Test(vec3f, set)
{
    struct ctx c;
    ctx_init(&c);
    vec3f_set(&c.v, 3.0, 5.0, 7.0);
    cr_assert_eq(c.v.x, 3.0);
    cr_assert_eq(c.v.y, 5.0);
    cr_assert_eq(c.v.z, 7.0);
}

Test(vec3f, zero)
{
    struct ctx c;
    ctx_init(&c);
    vec3f_zero(&c.v);
    cr_assert_eq(c.v.x, 0.0);
    cr_assert_eq(c.v.y, 0.0);
    cr_assert_eq(c.v.z, 0.0);
}

Test(vec3f, copy)
{
    struct ctx c;
    ctx_init(&c);
    vec3f_copy(&c.v, &c.v357);
    cr_assert_eq(c.v.x, 3.0);
    cr_assert_eq(c.v.y, 5.0);
    cr_assert_eq(c.v.z, 7.0);
}

Test(vec3f, add)
{
    struct ctx c;
    ctx_init(&c);
    vec3f_add(&c.v, &c.v123, &c.v357);
    cr_assert_eq(c.v.x, 4.0);
    cr_assert_eq(c.v.y, 7.0);
    cr_assert_eq(c.v.z, 10.0);
}

Test(vec3f, sub)
{
    struct ctx c;
    ctx_init(&c);
    vec3f_sub(&c.v, &c.v123, &c.v357);
    cr_assert_eq(c.v.x, -2.0);
    cr_assert_eq(c.v.y, -3.0);
    cr_assert_eq(c.v.z, -4.0);
}

Test(vec3f, neg)
{
    struct ctx c;
    ctx_init(&c);
    vec3f_neg(&c.v, &c.v123);
    cr_assert_eq(c.v.x, -1.0);
    cr_assert_eq(c.v.y, -2.0);
    cr_assert_eq(c.v.z, -3.0);
}

Test(vec3f, mul)
{
    struct ctx c;
    ctx_init(&c);
    vec3f_mul(&c.v, &c.v123, 5.0);
    cr_assert_eq(c.v.x, 5.0);
    cr_assert_eq(c.v.y, 10.0);
    cr_assert_eq(c.v.z, 15.0);
}

Test(vec3f, cl)
{
    struct ctx c;
    struct pin3f_s p;
    ctx_init(&c);
    vec3f_cl(&p, &c.v123, &c.v357);
    cr_assert_eq(p.p0, 34.0);
    cr_assert_eq(p.p12, -1.0);
    cr_assert_eq(p.p23, -1.0);
    cr_assert_eq(p.p31, 2.0);
}

Test(vec3f, mad2)
{
    struct ctx c;
    ctx_init(&c);
    vec3f_mad2(&c.v, &c.v123, 4.0, &c.v357, 8.0);
    cr_assert_eq(c.v.x, 28.0);
    cr_assert_eq(c.v.y, 48.0);
    cr_assert_eq(c.v.z, 68.0);
}

Test(vec3f, mad3)
{
    struct ctx c;
    ctx_init(&c);
    vec3f_mad3(&c.v, &c.v123, 4.0, &c.v357, 8.0, &c.v248, 9.0);
    cr_assert_eq(c.v.x, 46.0);
    cr_assert_eq(c.v.y, 84.0);
    cr_assert_eq(c.v.z, 140.0);
}

Test(vec3f, mad4)
{
    struct ctx c;
    ctx_init(&c);
    vec3f_mad4(&c.v, &c.v123, 4.0, &c.v357, 8.0, &c.v248, 9.0, &c.v159, 2.0);
    cr_assert_eq(c.v.x, 48.0);
    cr_assert_eq(c.v.y, 94.0);
    cr_assert_eq(c.v.z, 158.0);
}

Test(vec3f, mad5)
{
    struct ctx c;
    ctx_init(&c);
    vec3f_mad5(&c.v, &c.v123, 4.0, &c.v357, 8.0, &c.v248, 9.0, &c.v159, 2.0, &c.v789, 5.0);
    cr_assert_eq(c.v.x, 83.0);
    cr_assert_eq(c.v.y, 134.0);
    cr_assert_eq(c.v.z, 203.0);
}

Test(vec3f, dot)
{
    struct ctx c;
    double d;
    ctx_init(&c);
    d = vec3f_dot(&c.v123, &c.v357);
    cr_assert_eq(d, 34.0);
}

Test(vec3f, cross)
{
    struct ctx c;
    ctx_init(&c);
    vec3f_cross(&c.v, &c.v123, &c.v357);
    cr_assert_eq(c.v.x, -1.0);
    cr_assert_eq(c.v.y, 2.0);
    cr_assert_eq(c.v.z, -1.0);
}

Test(vec3f, cl_vs_cross_and_dot)
{
    struct ctx c;
    struct pin3f_s p;
    double d;
    ctx_init(&c);
    vec3f_cl(&p, &c.v123, &c.v357);
    vec3f_cross(&c.v, &c.v123, &c.v357);
    d = vec3f_dot(&c.v123, &c.v357);
    cr_assert_eq(p.p0, d);
    cr_assert_eq(p.p12, c.v.z);
    cr_assert_eq(p.p23, c.v.x);
    cr_assert_eq(p.p31, c.v.y);
}

Test(vec3f, unit)
{
    struct ctx c;
    ctx_init(&c);
    vec3f_unit(&c.v, &c.v123);
    cr_assert_eq(c.v.x, 1.0 / sqrt(14.0));
    cr_assert_eq(c.v.y, 2.0 / sqrt(14.0));
    cr_assert_eq(c.v.z, 3.0 / sqrt(14.0));
}

Test(vec3f, len)
{
    struct ctx c;
    double d;
    ctx_init(&c);
    d = vec3f_len(&c.v123);
    cr_assert_eq(d, sqrt(14.0));
}

Test(vec3f, sqlen)
{
    struct ctx c;
    double d;
    ctx_init(&c);
    d = vec3f_sqlen(&c.v123);
    cr_assert_eq(d, 14.0);
}

Test(vec3f, tr)
{
    struct ctx c;
    double d;
    ctx_init(&c);
    d = vec3f_tr(&c.v123);
    cr_assert_eq(d, 6.0);
}
