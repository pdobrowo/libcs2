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
#include "cs2/beziertreeqq4f.h"
#include "cs2/mem.h"
#include <stddef.h>
#include <assert.h>

static void beziertreenodeqq4f_eval(struct beziertreenodeqq4f_s *n)
{
    struct bezierqq4f_coeff_s c;

    /* eval coeffs */
    n->r->f(&c.c00, n->u0, n->v0, n->r->d);
    n->r->f(&c.c01, n->u0, n->v1, n->r->d);
    n->r->f(&c.c02, n->u0, n->v2, n->r->d);
    n->r->f(&c.c10, n->u1, n->v0, n->r->d);
    n->r->f(&c.c11, n->u1, n->v1, n->r->d);
    n->r->f(&c.c12, n->u1, n->v2, n->r->d);
    n->r->f(&c.c20, n->u2, n->v0, n->r->d);
    n->r->f(&c.c21, n->u2, n->v1, n->r->d);
    n->r->f(&c.c22, n->u2, n->v2, n->r->d);

    bezierqq4f_from_qq(&n->b, &c);
}

void beziertreenodeqq4f_init(struct beziertreenodeqq4f_s *n, double u0, double u1, double u2, double v0, double v1, double v2, struct beziertreeqq4f_s *t, struct beziertreenodeqq4f_s *pn, int is_virt)
{
    bezierqq4f_init(&n->b);

    n->u0 = u0;
    n->u1 = u1;
    n->u2 = u2;
    n->v0 = v0;
    n->v1 = v1;
    n->v2 = v2;

    n->r = t;
    n->p = pn;

    n->c[0][0] = 0;
    n->c[0][1] = 0;
    n->c[1][0] = 0;
    n->c[1][1] = 0;

    if (!is_virt)
    {
        beziertreenodeqq4f_eval(n);

        n->vol = n->b.h.vol;
        n->area = n->b.h.area;
    }
    else
    {
        n->vol = 0.0;
        n->area = 0.0;
    }
}

void beziertreenodeqq4f_clear(struct beziertreenodeqq4f_s *n)
{
    if (!n)
        return;

    beziertreenodeqq4f_clear(n->c[0][0]);
    MEM_FREE(n->c[0][0]);

    beziertreenodeqq4f_clear(n->c[0][1]);
    MEM_FREE(n->c[0][1]);

    beziertreenodeqq4f_clear(n->c[1][0]);
    MEM_FREE(n->c[1][0]);

    beziertreenodeqq4f_clear(n->c[1][1]);
    MEM_FREE(n->c[1][1]);

    bezierqq4f_clear(&n->b);
}

void beziertreenodeqq4f_sub(struct beziertreenodeqq4f_s *n)
{
    n->c[0][0] = MEM_MALLOC(struct beziertreenodeqq4f_s);
    n->c[0][1] = MEM_MALLOC(struct beziertreenodeqq4f_s);
    n->c[1][0] = MEM_MALLOC(struct beziertreenodeqq4f_s);
    n->c[1][1] = MEM_MALLOC(struct beziertreenodeqq4f_s);

    beziertreenodeqq4f_init(n->c[0][0], n->u0, 0.5 * (n->u1 + n->u0), n->u1, n->v0, 0.5 * (n->v1 + n->v0), n->v1, n->r, n, 0);
    beziertreenodeqq4f_init(n->c[0][1], n->u0, 0.5 * (n->u1 + n->u0), n->u1, n->v1, 0.5 * (n->v2 + n->v1), n->v2, n->r, n, 0);
    beziertreenodeqq4f_init(n->c[1][0], n->u1, 0.5 * (n->u2 + n->u1), n->u2, n->v0, 0.5 * (n->v1 + n->v0), n->v1, n->r, n, 0);
    beziertreenodeqq4f_init(n->c[1][1], n->u1, 0.5 * (n->u2 + n->u1), n->u2, n->v1, 0.5 * (n->v2 + n->v1), n->v2, n->r, n, 0);
}

int beziertreenodeqq4f_is_virt(struct beziertreenodeqq4f_s *n)
{
    return !n->p;
}

int beziertreenodeqq4f_is_leaf(struct beziertreenodeqq4f_s *n)
{
    assert((n->c[0][0] && n->c[0][1] && n->c[1][0] && n->c[1][1]) ||
           (!n->c[0][0] && !n->c[0][1] && !n->c[1][0] && !n->c[1][1]));

    return !n->c[0][0];
}

double beziertreenodeqq4f_vol(struct beziertreenodeqq4f_s *n)
{
    return beziertreenodeqq4f_is_leaf(n) ?
           n->vol :
           beziertreenodeqq4f_vol(n->c[0][0]) + beziertreenodeqq4f_vol(n->c[0][1]) + beziertreenodeqq4f_vol(n->c[1][0]) + beziertreenodeqq4f_vol(n->c[1][1]);
}

double beziertreenodeqq4f_area(struct beziertreenodeqq4f_s *n)
{
    return beziertreenodeqq4f_is_leaf(n) ?
           n->area :
           beziertreenodeqq4f_area(n->c[0][0]) + beziertreenodeqq4f_area(n->c[0][1]) + beziertreenodeqq4f_area(n->c[1][0]) + beziertreenodeqq4f_area(n->c[1][1]);
}

void beziertreeqq4f_init(struct beziertreeqq4f_s *t)
{
    t->f = 0;
    t->d = 0;
    t->rn = 0;
}

void beziertreeqq4f_clear(struct beziertreeqq4f_s *t)
{
    beziertreenodeqq4f_clear(t->rn);
    MEM_FREE(t->rn);
}

void beziertreeqq4f_from_func(struct beziertreeqq4f_s *t, beziertreeqq4f_func_t f, void *d)
{
    t->f = f;
    t->d = d;

    /* virtual */
    t->rn = MEM_MALLOC(struct beziertreenodeqq4f_s);

    beziertreenodeqq4f_init(t->rn, 0.0, 0.5, 1.0, 0.0, 0.5, 1.0, t, 0, 1);
}

double beziertreeqq4f_vol(struct beziertreeqq4f_s *t)
{
    return beziertreenodeqq4f_vol(t->rn);
}

double beziertreeqq4f_area(struct beziertreeqq4f_s *t)
{
    return beziertreenodeqq4f_area(t->rn);
}

static void beziertreeleafsqq4f_add(struct beziertreeleafsqq4f_s *l, struct beziertreenodeqq4f_s *n)
{
    struct beziertreeleafqq4f_s *nl = MEM_MALLOC(struct beziertreeleafqq4f_s);
    nl->n = n;
    nl->next = l->l;
    l->l = nl;
    ++l->c;
}

static void beziertreeleafsqq4f_init_r(struct beziertreeleafsqq4f_s *l, struct beziertreenodeqq4f_s *n)
{
    if (beziertreenodeqq4f_is_leaf(n))
        beziertreeleafsqq4f_add(l, n);
    else
    {
        beziertreeleafsqq4f_init_r(l, n->c[0][0]);
        beziertreeleafsqq4f_init_r(l, n->c[0][1]);
        beziertreeleafsqq4f_init_r(l, n->c[1][0]);
        beziertreeleafsqq4f_init_r(l, n->c[1][1]);
    }
}

void beziertreeleafsqq4f_init(struct beziertreeleafsqq4f_s *l, struct beziertreeqq4f_s *t)
{
    l->c = 0;
    l->l = 0;

    if (t->rn)
        beziertreeleafsqq4f_init_r(l, t->rn);
}

void beziertreeleafsqq4f_clear(struct beziertreeleafsqq4f_s *l)
{
    struct beziertreeleafqq4f_s *nll, *ll = l->l;

    while (ll)
    {
        nll = ll->next;
        MEM_FREE(ll);
        ll = nll;
    }
}

static size_t beziertreeleafsqq4f_sub_vol_i(struct beziertreeleafqq4f_s *l, double vol, struct beziertreeleafqq4f_s **nl)
{
    struct beziertreeleafqq4f_s *l01, *l10, *l11;

    if (!beziertreenodeqq4f_is_virt(l->n) && beziertreenodeqq4f_vol(l->n) <= vol)
    {
        *nl = (*nl)->next;
        return 0;
    }

    beziertreenodeqq4f_sub(l->n);

    l01 = MEM_MALLOC(struct beziertreeleafqq4f_s);
    l10 = MEM_MALLOC(struct beziertreeleafqq4f_s);
    l11 = MEM_MALLOC(struct beziertreeleafqq4f_s);

    l11->n = l->n->c[1][1];
    l10->n = l->n->c[1][0];
    l01->n = l->n->c[0][1];
    l->n = l->n->c[0][0];

    l11->next = l->next;
    l10->next = l11;
    l01->next = l10;
    l->next = l01;

    return 3;
}

void beziertreeleafsqq4f_sub_vol(struct beziertreeleafsqq4f_s *l, double vol)
{
    struct beziertreeleafqq4f_s *ll = l->l;

    while (ll)
        l->c += beziertreeleafsqq4f_sub_vol_i(ll, vol, &ll);
}
