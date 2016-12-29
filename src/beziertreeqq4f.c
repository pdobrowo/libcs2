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

static void beziertreenodeqq4f_eval(struct beziertreenodeqq4f_s *btn)
{
    struct bezierqq4f_coeff_s c;

    /* eval coeffs */
    btn->r->f(&c.c00, btn->u0, btn->v0, btn->r->data);
    btn->r->f(&c.c01, btn->u0, btn->v1, btn->r->data);
    btn->r->f(&c.c02, btn->u0, btn->v2, btn->r->data);
    btn->r->f(&c.c10, btn->u1, btn->v0, btn->r->data);
    btn->r->f(&c.c11, btn->u1, btn->v1, btn->r->data);
    btn->r->f(&c.c12, btn->u1, btn->v2, btn->r->data);
    btn->r->f(&c.c20, btn->u2, btn->v0, btn->r->data);
    btn->r->f(&c.c21, btn->u2, btn->v1, btn->r->data);
    btn->r->f(&c.c22, btn->u2, btn->v2, btn->r->data);

    bezierqq4f_from_qq(&btn->b, &c);
}

void beziertreenodeqq4f_init(struct beziertreenodeqq4f_s *btn, double u0, double u1, double u2, double v0, double v1, double v2, struct beziertreeqq4f_s *r, struct beziertreenodeqq4f_s *p, int is_virt)
{
    bezierqq4f_init(&btn->b);

    btn->u0 = u0;
    btn->u1 = u1;
    btn->u2 = u2;
    btn->v0 = v0;
    btn->v1 = v1;
    btn->v2 = v2;

    btn->r = r;
    btn->p = p;

    btn->c[0][0] = 0;
    btn->c[0][1] = 0;
    btn->c[1][0] = 0;
    btn->c[1][1] = 0;

    if (!is_virt)
    {
        beziertreenodeqq4f_eval(btn);

        btn->vol = btn->b.h.vol;
        btn->area = btn->b.h.area;
    }
    else
    {
        btn->vol = 0.0;
        btn->area = 0.0;
    }
}

void beziertreenodeqq4f_clear(struct beziertreenodeqq4f_s *btn)
{
    if (!btn)
        return;

    beziertreenodeqq4f_clear(btn->c[0][0]);
    beziertreenodeqq4f_clear(btn->c[0][1]);
    beziertreenodeqq4f_clear(btn->c[1][0]);
    beziertreenodeqq4f_clear(btn->c[1][1]);

    MEM_FREE(btn);

    bezierqq4f_clear(&btn->b);
}

void beziertreenodeqq4f_sub(struct beziertreenodeqq4f_s *btn)
{
    btn->c[0][0] = MEM_MALLOC(struct beziertreenodeqq4f_s);
    btn->c[0][1] = MEM_MALLOC(struct beziertreenodeqq4f_s);
    btn->c[1][0] = MEM_MALLOC(struct beziertreenodeqq4f_s);
    btn->c[1][1] = MEM_MALLOC(struct beziertreenodeqq4f_s);

    beziertreenodeqq4f_init(btn->c[0][0], btn->u0, 0.5 * (btn->u1 + btn->u0), btn->u1, btn->v0, 0.5 * (btn->v1 + btn->v0), btn->v1, btn->r, btn, 0);
    beziertreenodeqq4f_init(btn->c[0][1], btn->u0, 0.5 * (btn->u1 + btn->u0), btn->u1, btn->v1, 0.5 * (btn->v2 + btn->v1), btn->v2, btn->r, btn, 0);
    beziertreenodeqq4f_init(btn->c[1][0], btn->u1, 0.5 * (btn->u2 + btn->u1), btn->u2, btn->v0, 0.5 * (btn->v1 + btn->v0), btn->v1, btn->r, btn, 0);
    beziertreenodeqq4f_init(btn->c[1][1], btn->u1, 0.5 * (btn->u2 + btn->u1), btn->u2, btn->v1, 0.5 * (btn->v2 + btn->v1), btn->v2, btn->r, btn, 0);
}

int beziertreenodeqq4f_is_virt(struct beziertreenodeqq4f_s *btn)
{
    return !btn->p;
}

int beziertreenodeqq4f_is_leaf(struct beziertreenodeqq4f_s *btn)
{
    assert((btn->c[0][0] && btn->c[0][1] && btn->c[1][0] && btn->c[1][1]) ||
           (!btn->c[0][0] && !btn->c[0][1] && !btn->c[1][0] && !btn->c[1][1]));

    return !btn->c[0][0];
}

double beziertreenodeqq4f_vol(struct beziertreenodeqq4f_s *btn)
{
    return beziertreenodeqq4f_is_leaf(btn) ?
           btn->vol :
           beziertreenodeqq4f_vol(btn->c[0][0]) + beziertreenodeqq4f_vol(btn->c[0][1]) + beziertreenodeqq4f_vol(btn->c[1][0]) + beziertreenodeqq4f_vol(btn->c[1][1]);
}

double beziertreenodeqq4f_area(struct beziertreenodeqq4f_s *btn)
{
    return beziertreenodeqq4f_is_leaf(btn) ?
           btn->area :
           beziertreenodeqq4f_area(btn->c[0][0]) + beziertreenodeqq4f_area(btn->c[0][1]) + beziertreenodeqq4f_area(btn->c[1][0]) + beziertreenodeqq4f_area(btn->c[1][1]);
}

void beziertreeqq4f_init(struct beziertreeqq4f_s *bt)
{
    bt->f = 0;
    bt->data = 0;
    bt->r = 0;
}

void beziertreeqq4f_clear(struct beziertreeqq4f_s *bt)
{
    beziertreenodeqq4f_clear(bt->r);
}

void beziertreeqq4f_from_func(struct beziertreeqq4f_s *bt, beziertreeqq4f_func_t f, void *data)
{
    bt->f = f;
    bt->data = data;

    /* virtual */
    bt->r = MEM_MALLOC(struct beziertreenodeqq4f_s);

    beziertreenodeqq4f_init(bt->r, 0.0, 0.5, 1.0, 0.0, 0.5, 1.0, bt, 0, 1);
}

double beziertreeqq4f_vol(struct beziertreeqq4f_s *btn)
{
    return beziertreenodeqq4f_vol(btn->r);
}

double beziertreeqq4f_area(struct beziertreeqq4f_s *btn)
{
    return beziertreenodeqq4f_area(btn->r);
}
