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
#include "cs2/pin3f.h"
#include <math.h>

const pin3f_t PIN3F_ZERO = { 0.0, 0.0, 0.0, 0.0 };
const pin3f_t PIN3F_ONE = { 0.0, 0.0, 0.0, 1.0 };

void pin3f_set(pin3f_t *r, double p12, double p23, double p31, double p0)
{
    r->p12 = p12;
    r->p23 = p23;
    r->p31 = p31;
    r->p0 = p0;
}

void pin3f_zero(pin3f_t *r)
{
    pin3f_set(r, 0, 0, 0, 0);
}

void pin3f_add(pin3f_t *r, const pin3f_t *a, const pin3f_t *b)
{
    r->p12 = a->p12 + b->p12;
    r->p23 = a->p23 + b->p23;
    r->p31 = a->p31 + b->p31;
    r->p0 = a->p0 + b->p0;
}

void pin3f_sub(pin3f_t *r, const pin3f_t *a, const pin3f_t *b)
{
    r->p12 = a->p12 - b->p12;
    r->p23 = a->p23 - b->p23;
    r->p31 = a->p31 - b->p31;
    r->p0 = a->p0 - b->p0;
}

void pin3f_neg(pin3f_t *r, const pin3f_t *v)
{
    r->p12 = -v->p12;
    r->p23 = -v->p23;
    r->p31 = -v->p31;
    r->p0 = -v->p0;
}

void pin3f_mul(pin3f_t *r, const pin3f_t *a, double as)
{
    r->p12 = a->p12 * as;
    r->p23 = a->p23 * as;
    r->p31 = a->p31 * as;
    r->p0 = a->p0 * as;
}

void pin3f_mad2(pin3f_t *r, const pin3f_t *a, double as, const pin3f_t *b, double bs)
{
    r->p12 = a->p12 * as + b->p12 * bs;
    r->p23 = a->p23 * as + b->p23 * bs;
    r->p31 = a->p31 * as + b->p31 * bs;
    r->p0 = a->p0 * as + b->p0 * bs;
}

void pin3f_mad3(pin3f_t *r, const pin3f_t *a, double as, const pin3f_t *b, double bs, const pin3f_t *c, double cs)
{
    r->p12 = a->p12 * as + b->p12 * bs + c->p12 * cs;
    r->p23 = a->p23 * as + b->p23 * bs + c->p23 * cs;
    r->p31 = a->p31 * as + b->p31 * bs + c->p31 * cs;
    r->p0 = a->p0 * as + b->p0 * bs + c->p0 * cs;
}

void pin3f_mad4(pin3f_t *r, const pin3f_t *a, double as, const pin3f_t *b, double bs, const pin3f_t *c, double cs, const pin3f_t *d, double ds)
{
    r->p12 = a->p12 * as + b->p12 * bs + c->p12 * cs + d->p12 * ds;
    r->p23 = a->p23 * as + b->p23 * bs + c->p23 * cs + d->p23 * ds;
    r->p31 = a->p31 * as + b->p31 * bs + c->p31 * cs + d->p31 * ds;
    r->p0 = a->p0 * as + b->p0 * bs + c->p0 * cs + d->p0 * ds;
}

void pin3f_mad5(pin3f_t *r, const pin3f_t *a, double as, const pin3f_t *b, double bs, const pin3f_t *c, double cs, const pin3f_t *d, double ds, const pin3f_t *e, double es)
{
    r->p12 = a->p12 * as + b->p12 * bs + c->p12 * cs + d->p12 * ds + e->p12 * es;
    r->p23 = a->p23 * as + b->p23 * bs + c->p23 * cs + d->p23 * ds + e->p23 * es;
    r->p31 = a->p31 * as + b->p31 * bs + c->p31 * cs + d->p31 * ds + e->p31 * es;
    r->p0 = a->p0 * as + b->p0 * bs + c->p0 * cs + d->p0 * ds + e->p0 * es;
}

double pin3f_len(const pin3f_t *v)
{
    return sqrt(v->p12 * v->p12 + v->p23 * v->p23 + v->p31 * v->p31 + v->p0 * v->p0);
}

double pin3f_sqlen(const pin3f_t *v)
{
    return v->p12 * v->p12 + v->p23 * v->p23 + v->p31 * v->p31 + v->p0 * v->p0;
}
