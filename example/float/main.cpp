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
#include "cs2/spinquad3f.h"
#include "cs2/spin3f.h"
#include "cs2/timer.h"
#include <cstdio>
#include <cmath>

int main()
{
    uint64_t begin = cs2_timer_nsec();

    struct cs2_predg3f_s g;

    /* pred */
    cs2_vec3f_set(&g.k, 1.0, 2.0, 3.0);
    cs2_vec3f_set(&g.l, -1.0, 0.0, 2.0);
    cs2_vec3f_set(&g.a, 4.0, 2.0, -2.0);
    cs2_vec3f_set(&g.b, 0.0, -2.0, 3.0);
    g.c = 1.0;

    printf("predg3f: k={%.2f, %.2f, %.2f}, l={%.2f, %.2f, %.2f}, a={%.2f, %.2f, %.2f}, b={%.2f, %.2f, %.2f}, c=%.2f\n",
           g.k.x, g.k.y, g.k.z, g.l.x, g.l.y, g.l.z, g.a.x, g.a.y, g.a.z, g.b.x, g.b.y, g.b.z, g.c);

    /* pred type */
    cs2_predgtype3f_e gt = cs2_predg3f_type(&g);

    printf("predgtype3f: %s\n", cs2_predgtype3f_str(gt));

    /* spinquad */
    struct cs2_spinquad3f_s gsq;
    cs2_spinquad3f_from_predg3f(&gsq, &g);

    printf("spinquad3f: a11=%.2f, a22=%.2f, a33=%.2f, a44=%.2f, a12=%.2f, a13=%.2f, a14=%.2f, a23=%.2f, a24=%.2f, a34=%.2f\n",
           gsq.a11, gsq.a22, gsq.a33, gsq.a44, gsq.a12, gsq.a13, gsq.a14, gsq.a23, gsq.a24, gsq.a34);

    /* spinquad eval */
    struct cs2_spin3f_s s;
    cs2_spin3f_set(&s, 1.0 / sqrt(1.0 + 2.0 + 3.0 + 4.0),
                   2.0 / sqrt(1.0 + 2.0 + 3.0 + 4.0),
                   3.0 / sqrt(1.0 + 2.0 + 3.0 + 4.0),
                   4.0 / sqrt(1.0 + 2.0 + 3.0 + 4.0));

    double v = cs2_spinquad3f_eval(&gsq, &s);

    printf("spinquad3x_eval: %.2f\n", v);

    /* timing */
    uint64_t end = cs2_timer_nsec();

    printf("time: %lu ns\n", static_cast<unsigned long>(end - begin));

    return 0;
}

