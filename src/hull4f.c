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
#include "cs2/fmt.h"
#include "libqhull_r/qhull_ra.h"
#include <setjmp.h>
#include <assert.h>
#include <stdio.h>

static int hull4f_sep(const struct hull4f_s *h, const struct plane4f_s *p)
{
    size_t vi;

    for (vi = 0; vi < h->nvr; ++vi)
        if (plane4f_pops(p, &h->vr[vi]) < 0.0)
            return 0;

    return 1;
}

void hull4f_init(struct hull4f_s *h)
{
    h->hr = NULL;
    h->nhr = 0;
    h->vr = NULL;
    h->nvr = 0;
    h->vol = 0.0;
    h->area = 0.0;
}

void hull4f_clear(struct hull4f_s *h)
{
    MEM_FREE(h->hr);
    MEM_FREE(h->vr);
}

void hull4f_from_arr(struct hull4f_s *h, const struct vec4f_s *v, size_t c)
{
    int curlong, totlong, exitcode;
    double *pts;
    char opt[] = "qhull";
    struct qhT qh;
    struct vec4f_s n;
    facetT *fi;
    vertexT *vi;
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

        /* extra checks */
        assert(qh.hull_dim == 4);

        /* hull */
        h->nhr = (size_t)qh.num_facets;
        h->hr = MEM_MALLOC_N(struct plane4f_s, h->nhr);

        i = 0;

        for (fi = qh.facet_list; fi && fi->next; fi = fi->next)
        {
            vec4f_set(&n, fi->normal[0], fi->normal[1], fi->normal[2], fi->normal[3]);
            plane4f_set(&h->hr[i], &n, fi->offset);
            ++i;
        }

        h->nvr = (size_t)qh.num_vertices;
        h->vr = MEM_MALLOC_N(struct vec4f_s, h->nvr);

        i = 0;

        for (vi = qh.vertex_list; vi && vi->next; vi = vi->next)
        {
            vec4f_set(&h->vr[i], vi->point[0], vi->point[1], vi->point[2], vi->point[3]);
            ++i;
        }

        /* volume and area */
        qh_getarea(&qh, qh.facet_list);

        h->vol = qh.totvol;
        h->area = qh.totarea;
    }
    else
    {
        assert(0 && "qhull error - see stderr for details");
    }

    qh.NOerrexit = True;
    qh_freeqhull(&qh, !qh_ALL);
    qh_memfreeshort(&qh, &curlong, &totlong);

    assert(!curlong && !totlong && "qhull mem leak");
}

int hull4f_inter(const struct hull4f_s *p, const struct hull4f_s *q)
{
    size_t hi;

    for (hi = 0; hi < p->nhr; ++hi)
    {
        if (hull4f_sep(q, &p->hr[hi]))
            return 0;
    }

    for (hi = 0; hi < q->nhr; ++hi)
    {
        if (hull4f_sep(p, &q->hr[hi]))
            return 0;
    }

    return 1;
}

CS2_API void hull4f_print_json(struct hull4f_s *h, FILE *f, size_t ind)
{
    size_t i;

    fmt_indent(ind, f);
    fprintf(f, "{\n");

    fmt_indent(ind + FMT_DEFAULT_INDENT, f);
    fprintf(f, "\"h\":\n");

    fmt_indent(ind + FMT_DEFAULT_INDENT, f);
    fprintf(f, "[\n");

    for (i = 0; i < h->nhr; ++i)
    {
        plane4f_print_json(&h->hr[i], f, ind + FMT_DEFAULT_INDENT * 2);

        if (i != h->nhr - 1)
            fprintf(f, ",");

        fprintf(f, "\n");
    }

    fmt_indent(ind + FMT_DEFAULT_INDENT, f);
    fprintf(f, "],\n");

    fmt_indent(ind + FMT_DEFAULT_INDENT, f);
    fprintf(f, "\"v\":\n");

    fmt_indent(ind + FMT_DEFAULT_INDENT, f);
    fprintf(f, "[\n");

    for (i = 0; i < h->nvr; ++i)
    {
        vec4f_print_json(&h->vr[i], f, ind + FMT_DEFAULT_INDENT * 2);

        if (i != h->nvr - 1)
            fprintf(f, ",");

        fprintf(f, "\n");
    }

    fmt_indent(ind + FMT_DEFAULT_INDENT, f);
    fprintf(f, "]\n");

    fmt_indent(ind, f);
    fprintf(f, "}");
}