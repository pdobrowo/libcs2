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
#include "cs2/vec3f.h"
#include "cs2/fmt.h"
#include "cs2/assert.h"
#include <math.h>

void cs2_vec3f_set(struct cs2_vec3f_s *r, double x, double y, double z)
{
    r->x = x;
    r->y = y;
    r->z = z;
}

void cs2_vec3f_zero(struct cs2_vec3f_s *r)
{
    cs2_vec3f_set(r, 0, 0, 0);
}

double cs2_vec3f_coord(const struct cs2_vec3f_s *r, int k)
{
    switch (k) {
    case 0: return r->x;
    case 1: return r->y;
    case 2: return r->z;
    }
    CS2_PANIC_MSG("invalid coordinate");
    return 0.0;
}

void cs2_vec3f_copy(struct cs2_vec3f_s *r, const struct cs2_vec3f_s *v)
{
    r->x = v->x;
    r->y = v->y;
    r->z = v->z;
}

void cs2_vec3f_add(struct cs2_vec3f_s *r, const struct cs2_vec3f_s *a, const struct cs2_vec3f_s *b)
{
    r->x = a->x + b->x;
    r->y = a->y + b->y;
    r->z = a->z + b->z;
}

void cs2_vec3f_sub(struct cs2_vec3f_s *r, const struct cs2_vec3f_s *a, const struct cs2_vec3f_s *b)
{
    r->x = a->x - b->x;
    r->y = a->y - b->y;
    r->z = a->z - b->z;
}

void cs2_vec3f_neg(struct cs2_vec3f_s *r, const struct cs2_vec3f_s *v)
{
    r->x = -v->x;
    r->y = -v->y;
    r->z = -v->z;
}

void cs2_vec3f_mul(struct cs2_vec3f_s *r, const struct cs2_vec3f_s *a, double as)
{
    r->x = a->x * as;
    r->y = a->y * as;
    r->z = a->z * as;
}

void cs2_vec3f_mad2(struct cs2_vec3f_s *r, const struct cs2_vec3f_s *a, double as, const struct cs2_vec3f_s *b, double bs)
{
    r->x = a->x * as + b->x * bs;
    r->y = a->y * as + b->y * bs;
    r->z = a->z * as + b->z * bs;
}

void cs2_vec3f_mad3(struct cs2_vec3f_s *r, const struct cs2_vec3f_s *a, double as, const struct cs2_vec3f_s *b, double bs, const struct cs2_vec3f_s *c, double cs)
{
    r->x = a->x * as + b->x * bs + c->x * cs;
    r->y = a->y * as + b->y * bs + c->y * cs;
    r->z = a->z * as + b->z * bs + c->z * cs;
}

void cs2_vec3f_mad4(struct cs2_vec3f_s *r, const struct cs2_vec3f_s *a, double as, const struct cs2_vec3f_s *b, double bs, const struct cs2_vec3f_s *c, double cs, const struct cs2_vec3f_s *d, double ds)
{
    r->x = a->x * as + b->x * bs + c->x * cs + d->x * ds;
    r->y = a->y * as + b->y * bs + c->y * cs + d->y * ds;
    r->z = a->z * as + b->z * bs + c->z * cs + d->z * ds;
}

void cs2_vec3f_mad5(struct cs2_vec3f_s *r, const struct cs2_vec3f_s *a, double as, const struct cs2_vec3f_s *b, double bs, const struct cs2_vec3f_s *c, double cs, const struct cs2_vec3f_s *d, double ds, const struct cs2_vec3f_s *e, double es)
{
    r->x = a->x * as + b->x * bs + c->x * cs + d->x * ds + e->x * es;
    r->y = a->y * as + b->y * bs + c->y * cs + d->y * ds + e->y * es;
    r->z = a->z * as + b->z * bs + c->z * cs + d->z * ds + e->z * es;
}

double cs2_vec3f_dot(const struct cs2_vec3f_s *a, const struct cs2_vec3f_s *b)
{
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

void cs2_vec3f_cross(struct cs2_vec3f_s *r, const struct cs2_vec3f_s *a, const struct cs2_vec3f_s *b)
{
    r->x = a->y * b->z - a->z * b->y;
    r->y = a->z * b->x - a->x * b->z;
    r->z = a->x * b->y - a->y * b->x;
}

void cs2_vec3f_unit(struct cs2_vec3f_s *r, const struct cs2_vec3f_s *v)
{
    cs2_vec3f_mul(r, v, 1.0 / cs2_vec3f_len(v));
}

double cs2_vec3f_len(const struct cs2_vec3f_s *v)
{
    return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

double cs2_vec3f_sqlen(const struct cs2_vec3f_s *v)
{
    return v->x * v->x + v->y * v->y + v->z * v->z;
}

double cs2_vec3f_tr(const struct cs2_vec3f_s *v)
{
    return v->x + v->y + v->z;
}

void cs2_vec3f_print_json(const struct cs2_vec3f_s *r, FILE *f, size_t ind)
{
    cs2_fmt_indent(ind, f);
    fprintf(f, "{ \"x\": %.2f, \"y\": %.2f, \"z\": %.2f }", r->x, r->y, r->z);
}
