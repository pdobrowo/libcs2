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
#include "cs2/vec3x.h"

void vec3x_init(vec3x_t *v)
{
    mpz_init(v->x);
    mpz_init(v->y);
    mpz_init(v->z);
}

void vec3x_clear(vec3x_t *v)
{
    mpz_clear(v->x);
    mpz_clear(v->y);
    mpz_clear(v->z);
}

void vec3x_set(vec3x_t *r, const mpz_t x, const mpz_t y, const mpz_t z)
{
    mpz_set(r->x, x);
    mpz_set(r->y, y);
    mpz_set(r->z, z);
}

void vec3x_set_si(vec3x_t *r, long x, long y, long z)
{
    mpz_set_si(r->x, x);
    mpz_set_si(r->y, y);
    mpz_set_si(r->z, z);
}

void vec3x_zero(vec3x_t *r)
{
    mpz_set_si(r->x, 0);
    mpz_set_si(r->y, 0);
    mpz_set_si(r->z, 0);
}

int vec3x_is_zero(const vec3x_t *v)
{
    return !mpz_sgn(v->x) && !mpz_sgn(v->y) && !mpz_sgn(v->z);
}

void vec3x_copy(vec3x_t *r, const vec3x_t *v)
{
    mpz_set(r->x, v->x);
    mpz_set(r->y, v->y);
    mpz_set(r->z, v->z);
}

void vec3x_add(vec3x_t *r, const vec3x_t *a, const vec3x_t *b)
{
    mpz_add(r->x, a->x, b->x);
    mpz_add(r->y, a->y, b->y);
    mpz_add(r->z, a->z, b->z);
}

void vec3x_sub(vec3x_t *r, const vec3x_t *a, const vec3x_t *b)
{
    mpz_sub(r->x, a->x, b->x);
    mpz_sub(r->y, a->y, b->y);
    mpz_sub(r->z, a->z, b->z);
}

void vec3x_neg(vec3x_t *r, const vec3x_t *v)
{
    mpz_neg(r->x, v->x);
    mpz_neg(r->y, v->y);
    mpz_neg(r->z, v->z);
}

void vec3x_mul(vec3x_t *r, const vec3x_t *a, const mpz_t as)
{
    mpz_mul(r->x, a->x, as);
    mpz_mul(r->y, a->y, as);
    mpz_mul(r->z, a->z, as);
}

void vec3x_mad2(vec3x_t *r, const vec3x_t *a, const mpz_t as, const vec3x_t *b, const mpz_t bs)
{
    mpz_t t;
    mpz_init(t);
    mpz_mul(r->x, a->x, as);
    mpz_mul(t, b->x, bs);
    mpz_add(r->x, r->x, t);
    mpz_mul(r->y, a->y, as);
    mpz_mul(t, b->y, bs);
    mpz_add(r->y, r->y, t);
    mpz_mul(r->z, a->z, as);
    mpz_mul(t, b->z, bs);
    mpz_add(r->z, r->z, t);
    mpz_clear(t);
}

void vec3x_mad3(vec3x_t *r, const vec3x_t *a, const mpz_t as, const vec3x_t *b, const mpz_t bs, const vec3x_t *c, const mpz_t cs)
{
    mpz_t t;
    mpz_init(t);
    mpz_mul(r->x, a->x, as);
    mpz_mul(t, b->x, bs);
    mpz_add(r->x, r->x, t);
    mpz_mul(t, c->x, cs);
    mpz_add(r->x, r->x, t);
    mpz_mul(r->y, a->y, as);
    mpz_mul(t, b->y, bs);
    mpz_add(r->y, r->y, t);
    mpz_mul(t, c->y, cs);
    mpz_add(r->y, r->y, t);
    mpz_mul(r->z, a->z, as);
    mpz_mul(t, b->z, bs);
    mpz_add(r->z, r->z, t);
    mpz_mul(t, c->z, cs);
    mpz_add(r->z, r->z, t);
    mpz_clear(t);
}

void vec3x_mad4(vec3x_t *r, const vec3x_t *a, const mpz_t as, const vec3x_t *b, const mpz_t bs, const vec3x_t *c, const mpz_t cs, const vec3x_t *d, const mpz_t ds)
{
    mpz_t t;
    mpz_init(t);
    mpz_mul(r->x, a->x, as);
    mpz_mul(t, b->x, bs);
    mpz_add(r->x, r->x, t);
    mpz_mul(t, c->x, cs);
    mpz_add(r->x, r->x, t);
    mpz_mul(t, d->x, ds);
    mpz_add(r->x, r->x, t);
    mpz_mul(r->y, a->y, as);
    mpz_mul(t, b->y, bs);
    mpz_add(r->y, r->y, t);
    mpz_mul(t, c->y, cs);
    mpz_add(r->y, r->y, t);
    mpz_mul(t, d->y, ds);
    mpz_add(r->y, r->y, t);
    mpz_mul(r->z, a->z, as);
    mpz_mul(t, b->z, bs);
    mpz_add(r->z, r->z, t);
    mpz_mul(t, c->z, cs);
    mpz_add(r->z, r->z, t);
    mpz_mul(t, d->z, ds);
    mpz_add(r->z, r->z, t);
    mpz_clear(t);
}

void vec3x_mad5(vec3x_t *r, const vec3x_t *a, const mpz_t as, const vec3x_t *b, const mpz_t bs, const vec3x_t *c, const mpz_t cs, const vec3x_t *d, const mpz_t ds, const vec3x_t *e, const mpz_t es)
{
    mpz_t t;
    mpz_init(t);
    mpz_mul(r->x, a->x, as);
    mpz_mul(t, b->x, bs);
    mpz_add(r->x, r->x, t);
    mpz_mul(t, c->x, cs);
    mpz_add(r->x, r->x, t);
    mpz_mul(t, d->x, ds);
    mpz_add(r->x, r->x, t);
    mpz_mul(t, e->x, es);
    mpz_add(r->x, r->x, t);
    mpz_mul(r->y, a->y, as);
    mpz_mul(t, b->y, bs);
    mpz_add(r->y, r->y, t);
    mpz_mul(t, c->y, cs);
    mpz_add(r->y, r->y, t);
    mpz_mul(t, d->y, ds);
    mpz_add(r->y, r->y, t);
    mpz_mul(t, e->y, es);
    mpz_add(r->y, r->y, t);
    mpz_mul(r->z, a->z, as);
    mpz_mul(t, b->z, bs);
    mpz_add(r->z, r->z, t);
    mpz_mul(t, c->z, cs);
    mpz_add(r->z, r->z, t);
    mpz_mul(t, d->z, ds);
    mpz_add(r->z, r->z, t);
    mpz_mul(t, e->z, es);
    mpz_add(r->z, r->z, t);
    mpz_clear(t);
}

void vec3x_dot(mpz_t r, const vec3x_t *a, const vec3x_t *b)
{
    mpz_t t;
    mpz_init(t);
    mpz_mul(r, a->x, b->x);
    mpz_mul(t, a->y, b->y);
    mpz_add(r, r, t);
    mpz_mul(t, a->z, b->z);
    mpz_add(r, r, t);
    mpz_clear(t);
}

void vec3x_cross(vec3x_t *r, const vec3x_t *a, const vec3x_t *b)
{
    mpz_t t;
    mpz_init(t);
    mpz_mul(r->x, a->y, b->z);
    mpz_mul(t, a->z, b->y);
    mpz_sub(r->x, r->x, t);
    mpz_mul(r->y, a->z, b->x);
    mpz_mul(t, a->x, b->z);
    mpz_sub(r->y, r->y, t);
    mpz_mul(r->z, a->x, b->y);
    mpz_mul(t, a->y, b->x);
    mpz_sub(r->z, r->z, t);
    mpz_clear(t);
}

void vec3x_sqlen(mpz_t r, const vec3x_t *v)
{
    mpz_t t;
    mpz_init(t);
    mpz_mul(r, v->x, v->x);
    mpz_mul(t, v->y, v->y);
    mpz_add(r, r, t);
    mpz_mul(t, v->z, v->z);
    mpz_add(r, r, t);
    mpz_clear(t);
}

void vec3x_tr(mpz_t r, const vec3x_t *v)
{
    mpz_add(r, v->x, v->y);
    mpz_add(r, r, v->z);
}
