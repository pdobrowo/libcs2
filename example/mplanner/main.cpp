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
#include "cs2/vec3f.h"
#include "cs2/predg3f.h"
#include "cs2/beziertreeqq4f.h"
#include "cs2/timer.h"
#include "prefabs.h"
#include <stdint.h>
#include <cstdio>
#include <cmath>

struct predbb_func_s
{
    struct cs2_predg3f_s p;
    struct cs2_predgparam3f_s pp;
};

static void predbb_func(struct cs2_vec4f_s *r, double u, double v, void *data)
{
    struct predbb_func_s *f = (struct predbb_func_s *)data;
    struct cs2_spin3f_s s;
    cs2_predgparam3f_eval(&s, &f->pp, u, v, 0);
    r->x = s.s12;
    r->y = s.s23;
    r->z = s.s31;
    r->w = s.s0;
}

int main()
{
    struct cs2_beziertreeqq4f_s t;
    struct cs2_beziertreeleafsqq4f_s l;
    struct predbb_func_s f;
    uint64_t start;

    cs2_prefabs_predg3f_a_z_barrel_1(&f.p);

    cs2_predg3f_param(&f.pp, &f.p);

    cs2_beziertreeqq4f_init(&t);
    cs2_beziertreeqq4f_from_func(&t, &predbb_func, &f);

    cs2_beziertreeleafsqq4f_init(&l, &t);

    start = cs2_timer_msec();

    for (double v = 0.1; v > 0.00000001; v /= 10)
    {
        cs2_beziertreeleafsqq4f_sub_vol(&l, v);
        printf("target hull vol: %.12f, perc: %.12f, subs: %d, time: %d ms\n",
               v, 100 * l.c * v / (M_PI * M_PI / 2.0),
               (int)l.c, (int)(cs2_timer_msec() - start));
    }

    /* clear */
    cs2_beziertreeqq4f_clear(&t);
    return 0;
}
