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
#include "cs2/plane3f.h"
#include "cs2/fmt.h"

void cs2_plane3f_set(struct cs2_plane3f_s *p, const struct cs2_vec3f_s *n, double d)
{
    cs2_vec3f_copy(&p->n, n);
    p->d = d;
}

void cs2_plane3f_copy(struct cs2_plane3f_s *p, const struct cs2_plane3f_s *pa)
{
    cs2_vec3f_copy(&p->n, &pa->n);
    p->d = pa->d;
}

double cs2_plane3f_pops(const struct cs2_plane3f_s *p, const struct cs2_vec3f_s *vp)
{
    return cs2_vec3f_dot(&p->n, vp) + p->d;
}

void cs2_plane3f_print_json(const struct cs2_plane3f_s *p, FILE *f, size_t indent)
{
    cs2_fmt_indent(indent, f);
    fprintf(f, "{\n");

    cs2_fmt_indent(indent + CS2_FMT_DEFAULT_INDENT, f);
    fprintf(f, "\"n\": ");
    cs2_vec3f_print_json(&p->n, f, 0);

    fprintf(f, ",\n");

    cs2_fmt_indent(indent + CS2_FMT_DEFAULT_INDENT, f);
    fprintf(f, "\"d\": %.2f\n", p->d);

    cs2_fmt_indent(indent, f);
    fprintf(f, "}");
}
