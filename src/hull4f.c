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
#include "cs2/hull4f.h"
#include "cs2/mem.h"
#include "libqhull_r/qhull_ra.h"
#include <setjmp.h>
#include <assert.h>
#include <stdio.h>

void hull4f_init(struct hull4f_s *h)
{
    h->hp = NULL;
    h->nhp = 0;
    h->vol = 0.0;
}

void hull4f_clear(struct hull4f_s *h)
{
    MEM_FREE(h->hp);
}

void hull4f_from_arr(struct hull4f_s *h, const struct vec4f_s *v, size_t c)
{
    int curlong, totlong, exitcode;
    double *pts;
    char opt[] = "qhull QJ"; /* enable 'joggle' */
    struct qhT qh;
    struct vec4f_s n;
    facetT *f;
    int i;

    /* qhull lib check */
    QHULL_LIB_CHECK

    /* pts */
    assert(sizeof(struct vec4f_s) == sizeof(double) * 4);
    pts = (double *)v;

    /* init */
    qh_init_A(&qh, stdin, stdout, stderr, 0, NULL);
    exitcode = setjmp(qh.errexit);

    if (!exitcode)
    {
        qh.NOerrexit = False;
        qh_initflags(&qh, opt);
        qh_init_B(&qh, pts, c, 4, False);
        qh_qhull(&qh);
        qh_check_output(&qh);

        /* hull */
        h->nhp = (size_t)qh.num_facets;
        h->hp = MEM_MALLOC_N(struct plane4f_s, h->nhp);

        assert(qh.hull_dim == 4);

        i = 0;

        for (f = qh.facet_list; f && f->next; f = f->next)
        {
            vec4f_set(&n, f->normal[0], f->normal[1], f->normal[2], f->normal[3]);
            plane4f_set(&h->hp[i], &n, f->offset);
            ++i;
        }

        /* vol */
        qh_getarea(&qh, qh.facet_list);
        h->vol = qh.totvol;
    }
    else
    {
        assert(0 && "qhull error");
    }

    qh.NOerrexit = True;
    qh_freeqhull(&qh, !qh_ALL);
    qh_memfreeshort(&qh, &curlong, &totlong);

    assert(!curlong && !totlong && "qhull mem leak");
}
