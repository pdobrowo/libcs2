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
#include "cs2/predg3x.h"
#include <cs2/assert.h>

static void _cs2_calc_r(struct cs2_vec3x_s *r, const struct cs2_vec3x_s *v)
{
    if (mpz_sgn(v->x))
    {
        /* take x-y plane */
        cs2_vec3x_set(r, v->y, v->x, v->z);
        mpz_neg(r->x, r->x);
    }
    else if (mpz_sgn(v->y))
    {
        /* take y-z plane */
        cs2_vec3x_set(r, v->x, v->z, v->y);
        mpz_neg(r->y, r->y);
    }
    else
    {
        /* take z-x plane */
        cs2_vec3x_set(r, v->z, v->y, v->x);
        mpz_neg(r->z, r->z);
    }
}

void cs2_predg3x_init(struct cs2_predg3x_s *g)
{
    cs2_vec3x_init(&g->k);
    cs2_vec3x_init(&g->l);
    cs2_vec3x_init(&g->a);
    cs2_vec3x_init(&g->b);
    mpz_init(g->c);
}

void cs2_predg3x_clear(struct cs2_predg3x_s *g)
{
    cs2_vec3x_clear(&g->k);
    cs2_vec3x_clear(&g->l);
    cs2_vec3x_clear(&g->a);
    cs2_vec3x_clear(&g->b);
    mpz_clear(g->c);
}

void cs2_predg3x_from_predh3x(struct cs2_predg3x_s *g, const struct cs2_predh3x_s *h)
{
    _cs2_calc_r(&g->l, &h->p.n);
    cs2_vec3x_cross(&g->k, &h->p.n, &g->l);
    cs2_vec3x_cross(&g->l, &h->p.n, &g->k);
    cs2_vec3x_copy(&g->a, &h->b);
    cs2_vec3x_neg(&g->b, &h->b);
    cs2_vec3x_sqlen(g->c, &g->k);
    mpz_mul(g->c, g->c, h->p.d);
    mpz_mul_2exp(g->c, g->c, 1);
}

void cs2_predg3x_from_preds3x(struct cs2_predg3x_s *g, const struct cs2_preds3x_s *s)
{
    cs2_vec3x_copy(&g->k, &s->k);
    cs2_vec3x_copy(&g->l, &s->l);
    cs2_vec3x_copy(&g->a, &s->a);
    cs2_vec3x_copy(&g->b, &s->b);
    mpz_set_si(g->c, 0);
}

void cs2_predg3x_pquv(struct cs2_vec3x_s *p, struct cs2_vec3x_s *q, struct cs2_vec3x_s *u, struct cs2_vec3x_s *v, const struct cs2_predg3x_s *g)
{
    cs2_vec3x_cross(p, &g->k, &g->l);
    cs2_vec3x_sub(q, &g->a, &g->b);
    cs2_vec3x_sub(u, &g->k, &g->l);
    cs2_vec3x_cross(v, &g->a, &g->b);
}

const char *cs2_predgtype3x_str(enum cs2_predgtype3x_e pgt)
{
    switch (pgt)
    {
    case cs2_predgtype3x_improper: return "improper";
    case cs2_predgtype3x_ellipsoidal: return "ellipsoidal";
    case cs2_predgtype3x_toroidal: return "toroidal";

    /* COUNT */
    case cs2_predgtype3x_COUNT: return 0;
    }

    return 0;
}

enum cs2_predgtype3x_e cs2_predg3x_type(const struct cs2_predg3x_s *pg)
{
    struct cs2_vec3x_s p, q, u, v;
    int pq, uv;
    enum cs2_predgtype3x_e t;
    cs2_vec3x_init(&p);
    cs2_vec3x_init(&q);
    cs2_vec3x_init(&u);
    cs2_vec3x_init(&v);
    cs2_predg3x_pquv(&p, &q, &u, &v, pg);
    pq = !cs2_vec3x_is_zero(&p) && !cs2_vec3x_is_zero(&q);
    uv = !cs2_vec3x_is_zero(&u) && !cs2_vec3x_is_zero(&v);
    if (pq && uv)
        t = cs2_predgtype3x_ellipsoidal;
    else if (pq || uv)
        t = cs2_predgtype3x_toroidal;
    else
        t = cs2_predgtype3x_improper;
    cs2_vec3x_clear(&p);
    cs2_vec3x_clear(&q);
    cs2_vec3x_clear(&u);
    cs2_vec3x_clear(&v);
    return t;
}
