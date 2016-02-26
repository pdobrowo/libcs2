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
#include "cs2/predg3x.h"
#include "cs2/spinquad3x.h"
#include "cs2/pin3x.h"
#include "cs2/timer.h"
#include <gmp.h>
#include <cstdio>

int main()
{
    uint64_t begin = timer_nsec();

    predg3x_t g;

    /* pred */
    predg3x_init(&g);
    vec3x_set_si(&g.k, 1, 2, 3);
    vec3x_set_si(&g.l, -1, 0, 2);
    vec3x_set_si(&g.a, 4, 2, -2);
    vec3x_set_si(&g.b, 0, -2, 3);
    mpz_set_si(g.c, 1);

    gmp_printf("predg3x: k={%Zd, %Zd, %Zd}, l={%Zd, %Zd, %Zd}, a={%Zd, %Zd, %Zd}, b={%Zd, %Zd, %Zd}, c=%Zd\n",
               g.k.x, g.k.y, g.k.z, g.l.x, g.l.y, g.l.z, g.a.x, g.a.y, g.a.z, g.b.x, g.b.y, g.b.z, g.c);

    /* pred type */
    predgtype3x_e gt = predg3x_type(&g);

    printf("predgtype3x: %s\n", predgtype3x_str(gt));

    /* spinquad */
    spinquad3x_t gsq;
    spinquad3x_init(&gsq);

    spinquad3x_from_predg3x(&gsq, &g);

    gmp_printf("spinquad3x: a11=%Zd, a22=%Zd, a33=%Zd, a44=%Zd, a12=%Zd, a13=%Zd, a14=%Zd, a23=%Zd, a24=%Zd, a34=%Zd\n",
               gsq.a11, gsq.a22, gsq.a33, gsq.a44, gsq.a12, gsq.a13, gsq.a14, gsq.a23, gsq.a24, gsq.a34);

    /* spinquad eval */
    pin3x_t p;
    pin3x_init(&p);
    pin3x_set_si(&p, 1, 2, 3, 4);

    mpz_t v;
    mpz_init(v);

    spinquad3x_eval(v, &gsq, &p);

    gmp_printf("spinquad3x_eval: %Zd\n", v);

    /* clear */
    mpz_clear(v);
    pin3x_clear(&p);
    spinquad3x_clear(&gsq);
    predg3x_clear(&g);

    /* timing */
    uint64_t end = timer_nsec();

    printf("time: %lu ns\n", static_cast<unsigned long>(end - begin));
    return 0;
}

