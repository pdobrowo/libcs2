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
#include "cs2/bezierqq1f.h"

void bezierqq1f_from_qq(struct bezierqq1f_s *b, const struct bezierqq1f_coeff_s *c)
{
    /* corners */
    b->p00 = c->c00;
    b->p02 = c->c02;
    b->p20 = c->c20;
    b->p22 = c->c22;

    /* mid-border */
    b->p01 = 0.5 * (4.0 * c->c01 - (b->p00 + b->p02));
    b->p10 = 0.5 * (4.0 * c->c10 - (b->p00 + b->p20));
    b->p21 = 0.5 * (4.0 * c->c21 - (b->p20 + b->p22));
    b->p12 = 0.5 * (4.0 * c->c12 - (b->p02 + b->p22));

    /* mid */
    b->p11 =  0.25 * (16.0 * c->c11 - 2.0 * (b->p01 + b->p10 + b->p12 + b->p21) - (b->p00 + b->p02 + b->p20 + b->p22));
}

double bezierqq1f_eval(const struct bezierqq1f_s *b, double u, double v)
{
    double uu = u * u;
    double cu = 1.0 - u;
    double cucu = cu * cu;
    double ucu = u  * cu;
    double vv = v * v;
    double cv = 1.0 - v;
    double cvcv = cv * cv;
    double vcv = v * cv;

    return 1.0 * 1.0 * cucu * cvcv * b->p00 + 1.0 * 2.0 * cucu * vcv * b->p01 + 1.0 * 1.0 * cucu * vv * b->p02 +
           2.0 * 1.0 * ucu  * cvcv * b->p10 + 2.0 * 2.0 * ucu  * vcv * b->p11 + 2.0 * 1.0 * ucu  * vv * b->p12 +
           1.0 * 1.0 * uu   * cvcv * b->p20 + 1.0 * 2.0 * uu   * vcv * b->p21 + 1.0 * 1.0 * uu   * vv * b->p22;
}
