/**
 * Copyright (c) 2015 Przemysław Dobrowolski
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
#include "cs2/predg3f.h"

static void choose_r_vector(vec3f_t *r, const vec3f_t *v)
{
    if (v->x != 0)
        vec3f_set(r, -v->y, v->x, v->z); /* take x-y plane */
    else if (v->y != 0)
        vec3f_set(r, v->x, -v->z, v->y); /* take y-z plane */
    else
        vec3f_set(r, v->z, v->y, -v->x); /* take z-x plane */
}

void predg3f_from_predh3f(predg3f_t *g, const predh3f_t *h)
{
    vec3f_t r, nr;

    choose_r_vector(&r, &h->p.n);
    vec3f_cross(&nr, &h->p.n, &r);

    g->k = nr;
    vec3f_cross(&g->l, &h->p.n, &nr);
    g->a = h->b;
    vec3f_neg(&g->b, &h->b);
    g->c = 2 * h->p.d * vec3f_sqlen(&nr);
}

void predg3f_from_preds3f(predg3f_t *g, const preds3f_t *s)
{
    g->k = s->k;
    g->l = s->l;
    g->a = s->a;
    g->b = s->b;
    g->c = 0;
}
