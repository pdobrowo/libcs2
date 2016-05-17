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
#include <math.h>

void vec3f_set(vec3f_t *r, double x, double y, double z)
{
    r->x = x;
    r->y = y;
    r->z = z;
}

void vec3f_zero(vec3f_t *r)
{
    vec3f_set(r, 0, 0, 0);
}

void vec3f_copy(vec3f_t *r, const vec3f_t *v)
{
    r->x = v->x;
    r->y = v->y;
    r->z = v->z;
}

void vec3f_add(vec3f_t *r, const vec3f_t *a, const vec3f_t *b)
{
    r->x = a->x + b->x;
    r->y = a->y + b->y;
    r->z = a->z + b->z;
}

void vec3f_sub(vec3f_t *r, const vec3f_t *a, const vec3f_t *b)
{
    r->x = a->x - b->x;
    r->y = a->y - b->y;
    r->z = a->z - b->z;
}

void vec3f_neg(vec3f_t *r, const vec3f_t *v)
{
    r->x = -v->x;
    r->y = -v->y;
    r->z = -v->z;
}

void vec3f_mul(vec3f_t *r, const vec3f_t *a, double as)
{
    r->x = a->x * as;
    r->y = a->y * as;
    r->z = a->z * as;
}

void vec3f_cl(pin3f_t *r, const vec3f_t *a, const vec3f_t *b)
{
    r->p12 = a->x * b->y - a->y * b->x;
    r->p23 = a->y * b->z - a->z * b->y;
    r->p31 = a->z * b->x - a->x * b->z;
    r->p0 = a->x * b->x + a->y * b->y + a->z * b->z;
}

void vec3f_mad2(vec3f_t *r, const vec3f_t *a, double as, const vec3f_t *b, double bs)
{
    r->x = a->x * as + b->x * bs;
    r->y = a->y * as + b->y * bs;
    r->z = a->z * as + b->z * bs;
}

void vec3f_mad3(vec3f_t *r, const vec3f_t *a, double as, const vec3f_t *b, double bs, const vec3f_t *c, double cs)
{
    r->x = a->x * as + b->x * bs + c->x * cs;
    r->y = a->y * as + b->y * bs + c->y * cs;
    r->z = a->z * as + b->z * bs + c->z * cs;
}

void vec3f_mad4(vec3f_t *r, const vec3f_t *a, double as, const vec3f_t *b, double bs, const vec3f_t *c, double cs, const vec3f_t *d, double ds)
{
    r->x = a->x * as + b->x * bs + c->x * cs + d->x * ds;
    r->y = a->y * as + b->y * bs + c->y * cs + d->y * ds;
    r->z = a->z * as + b->z * bs + c->z * cs + d->z * ds;
}

void vec3f_mad5(vec3f_t *r, const vec3f_t *a, double as, const vec3f_t *b, double bs, const vec3f_t *c, double cs, const vec3f_t *d, double ds, const vec3f_t *e, double es)
{
    r->x = a->x * as + b->x * bs + c->x * cs + d->x * ds + e->x * es;
    r->y = a->y * as + b->y * bs + c->y * cs + d->y * ds + e->y * es;
    r->z = a->z * as + b->z * bs + c->z * cs + d->z * ds + e->z * es;
}

double vec3f_dot(const vec3f_t *a, const vec3f_t *b)
{
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

void vec3f_cross(vec3f_t *r, const vec3f_t *a, const vec3f_t *b)
{
    r->x = a->y * b->z - a->z * b->y;
    r->y = a->z * b->x - a->x * b->z;
    r->z = a->x * b->y - a->y * b->x;
}

void vec3f_unit(vec3f_t *r, const vec3f_t *v)
{
    vec3f_mul(r, v, 1.0 / vec3f_len(v));
}

double vec3f_len(const vec3f_t *v)
{
    return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

double vec3f_sqlen(const vec3f_t *v)
{
    return v->x * v->x + v->y * v->y + v->z * v->z;
}

double vec3f_tr(const vec3f_t *v)
{
    return v->x + v->y + v->z;
}
