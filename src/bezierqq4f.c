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
#include "cs2/bezierqq4f.h"
#include "cs2/hull4f.h"

static void _cs2_bezierqq4f_calc_hull(struct cs2_bezierqq4f_s *b)
{
    struct cs2_vec4f_s pts[9];

    cs2_vec4f_copy(&pts[0], &b->p00);
    cs2_vec4f_copy(&pts[1], &b->p01);
    cs2_vec4f_copy(&pts[2], &b->p02);
    cs2_vec4f_copy(&pts[3], &b->p10);
    cs2_vec4f_copy(&pts[4], &b->p11);
    cs2_vec4f_copy(&pts[5], &b->p12);
    cs2_vec4f_copy(&pts[6], &b->p20);
    cs2_vec4f_copy(&pts[7], &b->p21);
    cs2_vec4f_copy(&pts[8], &b->p22);

    cs2_hull4f_from_arr(&b->h, pts, 9);
}

void cs2_bezierqq4f_init(struct cs2_bezierqq4f_s *b)
{
    cs2_hull4f_init(&b->h);
}

void cs2_bezierqq4f_clear(struct cs2_bezierqq4f_s *b)
{
    cs2_hull4f_clear(&b->h);
}

void cs2_bezierqq4f_from_qq(struct cs2_bezierqq4f_s *b, const struct cs2_bezierqq4f_coeff_s *c)
{
    /* corners */
    cs2_vec4f_copy(&b->p00, &c->c00);
    cs2_vec4f_copy(&b->p02, &c->c02);
    cs2_vec4f_copy(&b->p20, &c->c20);
    cs2_vec4f_copy(&b->p22, &c->c22);

    /* mid-border */
    #define BEZIERQQ44F_BORDER_CASE_IMPL(Coord) \
        b->p01.Coord = 0.5 * (4.0 * c->c01.Coord - (b->p00.Coord + b->p02.Coord)); \
        b->p10.Coord = 0.5 * (4.0 * c->c10.Coord - (b->p00.Coord + b->p20.Coord)); \
        b->p21.Coord = 0.5 * (4.0 * c->c21.Coord - (b->p20.Coord + b->p22.Coord)); \
        b->p12.Coord = 0.5 * (4.0 * c->c12.Coord - (b->p02.Coord + b->p22.Coord));

    BEZIERQQ44F_BORDER_CASE_IMPL(x)
    BEZIERQQ44F_BORDER_CASE_IMPL(y)
    BEZIERQQ44F_BORDER_CASE_IMPL(z)
    BEZIERQQ44F_BORDER_CASE_IMPL(w)
    #undef BEZIERQQ44F_BORDER_CASE_IMPL

    /* mid */
    #define BEZIERQQ44F_MID_CASE_IMPL(Coord) \
        b->p11.Coord =  0.25 * (16.0 * c->c11.Coord - 2.0 * (b->p01.Coord + b->p10.Coord + b->p12.Coord + b->p21.Coord) - (b->p00.Coord + b->p02.Coord + b->p20.Coord + b->p22.Coord));

    BEZIERQQ44F_MID_CASE_IMPL(x)
    BEZIERQQ44F_MID_CASE_IMPL(y)
    BEZIERQQ44F_MID_CASE_IMPL(z)
    BEZIERQQ44F_MID_CASE_IMPL(w)
    #undef BEZIERQQ44F_MID_CASE_IMPL

    /* hull */
    _cs2_bezierqq4f_calc_hull(b);
}

void cs2_bezierqq4f_eval(struct cs2_vec4f_s *r, const struct cs2_bezierqq4f_s *b, double u, double v)
{
    double uu = u * u;
    double cu = 1.0 - u;
    double cucu = cu * cu;
    double ucu = u  * cu;
    double vv = v * v;
    double cv = 1.0 - v;
    double cvcv = cv * cv;
    double vcv = v * cv;

    #define BEZIERQQ44F_EVAL_CASE_IMPL(Coord) \
        r->Coord = 1.0 * 1.0 * cucu * cvcv * b->p00.Coord + 1.0 * 2.0 * cucu * vcv * b->p01.Coord + 1.0 * 1.0 * cucu * vv * b->p02.Coord + \
                   2.0 * 1.0 * ucu  * cvcv * b->p10.Coord + 2.0 * 2.0 * ucu  * vcv * b->p11.Coord + 2.0 * 1.0 * ucu  * vv * b->p12.Coord + \
                   1.0 * 1.0 * uu   * cvcv * b->p20.Coord + 1.0 * 2.0 * uu   * vcv * b->p21.Coord + 1.0 * 1.0 * uu   * vv * b->p22.Coord;

    BEZIERQQ44F_EVAL_CASE_IMPL(x)
    BEZIERQQ44F_EVAL_CASE_IMPL(y)
    BEZIERQQ44F_EVAL_CASE_IMPL(z)
    BEZIERQQ44F_EVAL_CASE_IMPL(w)
    #undef BEZIERQQ44F_EVAL_CASE_IMPL
}

int cs2_bezierqq4f_inter(const struct cs2_bezierqq4f_s *p, const struct cs2_bezierqq4f_s *q)
{
    return cs2_hull4f_inter(&p->h, &q->h);
}
