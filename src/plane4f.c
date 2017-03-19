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
#include "cs2/plane4f.h"
#include "cs2/fmt.h"

void plane4f_set(struct plane4f_s *p, const struct vec4f_s *n, double d)
{
    vec4f_copy(&p->n, n);
    p->d = d;
}

void plane4f_copy(struct plane4f_s *r, const struct plane4f_s *p)
{
    vec4f_copy(&r->n, &p->n);
    r->d = p->d;
}

double plane4f_pops(const struct plane4f_s *r, const struct vec4f_s *p)
{
    return vec4f_dot(&r->n, p) + r->d;
}

void plane4f_print_json(const struct plane4f_s *r, FILE *f, size_t ind)
{
    fmt_indent(ind, f);
    fprintf(f, "{\n");

    fmt_indent(ind + FMT_DEFAULT_INDENT, f);
    fprintf(f, "\"n\": ");
    vec4f_print_json(&r->n, f, 0);

    fprintf(f, ",\n");

    fmt_indent(ind + FMT_DEFAULT_INDENT, f);
    fprintf(f, "\"d\": %.2f\n", r->d);

    fmt_indent(ind, f);
    fprintf(f, "}");
}
