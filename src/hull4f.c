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
#include "cs2/hull4f.h"
#include "cs2/mem.h"
#include "cs2/fmt.h"
#include "cs2/assert.h"
#include "libqhull_r/qhull_ra.h"
#include <setjmp.h>
#include <cs2/assert.h>
#include <stdio.h>

static int _cs2_hull4f_sep(const struct cs2_hull4f_s *h, const struct cs2_plane4f_s *p)
{
    size_t vi;

    for (vi = 0; vi < h->nvr; ++vi)
        if (cs2_plane4f_pops(p, &h->vr[vi]) < 0.0)
            return 0;

    return 1;
}

void cs2_hull4f_init(struct cs2_hull4f_s *h)
{
    h->hr = NULL;
    h->nhr = 0;
    h->vr = NULL;
    h->nvr = 0;
    h->vol = 0.0;
    h->area = 0.0;
}

void cs2_hull4f_clear(struct cs2_hull4f_s *h)
{
    CS2_MEM_FREE(h->hr);
    CS2_MEM_FREE(h->vr);
}

void cs2_hull4f_from_arr(struct cs2_hull4f_s *h, const struct cs2_vec4f_s *v, size_t n)
{
    int curlong, totlong, exitcode;
    const double *pts = 0;
    char opt[] = "qhull";
    struct qhT qh;
    struct cs2_vec4f_s vn;
    facetT *fi;
    vertexT *vi;
    int i;

    /* qhull lib check */
    QHULL_LIB_CHECK

    /* pts */
    CS2_ASSERT(sizeof(struct cs2_vec4f_s) == sizeof(double) * 4);
    pts = (const double *)v;

    /* init */
    qh_init_A(&qh, stdin, stdout, stderr, 0, NULL);
    exitcode = setjmp(qh.errexit);

    if (!exitcode)
    {
        qh.NOerrexit = False;
        qh_initflags(&qh, opt);
        qh_init_B(&qh, (double *)pts, (int)n, 4, False); /* TODO: fix dropped const qualifier */
        qh_qhull(&qh);
        qh_check_output(&qh);

        /* extra checks */
        CS2_ASSERT_MSG(qh.hull_dim == 4, "hull must be 4-dimensional");

        /* hull */
        h->nhr = (size_t)qh.num_facets;
        h->hr = CS2_MEM_MALLOC_N(struct cs2_plane4f_s, h->nhr);

        i = 0;

        for (fi = qh.facet_list; fi && fi->next; fi = fi->next)
        {
            cs2_vec4f_set(&vn, fi->normal[0], fi->normal[1], fi->normal[2], fi->normal[3]);
            cs2_plane4f_set(&h->hr[i], &vn, fi->offset);
            ++i;
        }

        h->nvr = (size_t)qh.num_vertices;
        h->vr = CS2_MEM_MALLOC_N(struct cs2_vec4f_s, h->nvr);

        i = 0;

        for (vi = qh.vertex_list; vi && vi->next; vi = vi->next)
        {
            cs2_vec4f_set(&h->vr[i], vi->point[0], vi->point[1], vi->point[2], vi->point[3]);
            ++i;
        }

        /* volume and area */
        qh_getarea(&qh, qh.facet_list);

        h->vol = qh.totvol;
        h->area = qh.totarea;
    }
    else
    {
        CS2_PANIC_MSG("qhull error - see stderr for details");
    }

    qh.NOerrexit = True;
    qh_freeqhull(&qh, !qh_ALL);
    qh_memfreeshort(&qh, &curlong, &totlong);

    CS2_ASSERT_MSG(!curlong && !totlong, "qhull mem leak");
}

int cs2_hull4f_inter(const struct cs2_hull4f_s *ha, const struct cs2_hull4f_s *hb)
{
    size_t i;

    for (i = 0; i < ha->nhr; ++i)
    {
        if (_cs2_hull4f_sep(hb, &ha->hr[i]))
            return 0;
    }

    for (i = 0; i < hb->nhr; ++i)
    {
        if (_cs2_hull4f_sep(ha, &hb->hr[i]))
            return 0;
    }

    return 1;
}

void cs2_hull4f_print_json(struct cs2_hull4f_s *h, FILE *f, size_t indent)
{
    size_t i;

    cs2_fmt_indent(indent, f);
    fprintf(f, "{\n");

    cs2_fmt_indent(indent + CS2_FMT_DEFAULT_INDENT, f);
    fprintf(f, "\"h\":\n");

    cs2_fmt_indent(indent + CS2_FMT_DEFAULT_INDENT, f);
    fprintf(f, "[\n");

    for (i = 0; i < h->nhr; ++i)
    {
        cs2_plane4f_print_json(&h->hr[i], f, indent + CS2_FMT_DEFAULT_INDENT * 2);

        if (i != h->nhr - 1)
            fprintf(f, ",");

        fprintf(f, "\n");
    }

    cs2_fmt_indent(indent + CS2_FMT_DEFAULT_INDENT, f);
    fprintf(f, "],\n");

    cs2_fmt_indent(indent + CS2_FMT_DEFAULT_INDENT, f);
    fprintf(f, "\"v\":\n");

    cs2_fmt_indent(indent + CS2_FMT_DEFAULT_INDENT, f);
    fprintf(f, "[\n");

    for (i = 0; i < h->nvr; ++i)
    {
        cs2_vec4f_print_json(&h->vr[i], f, indent + CS2_FMT_DEFAULT_INDENT * 2);

        if (i != h->nvr - 1)
            fprintf(f, ",");

        fprintf(f, "\n");
    }

    cs2_fmt_indent(indent + CS2_FMT_DEFAULT_INDENT, f);
    fprintf(f, "]\n");

    cs2_fmt_indent(indent, f);
    fprintf(f, "}");
}
