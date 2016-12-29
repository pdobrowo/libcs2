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
#ifndef CS2_BEZIERQQ1F_H
#define CS2_BEZIERQQ1F_H

#include "defs.h"

CS2_API_BEGIN

/**
 * bezier quadratic-quadratic patch in 1 dimension
 *
 * index (uv):
 *
 * 00 01 02
 * 10 11 12
 * 20 21 22
 */
struct bezierqq1f_s
{
    double p00, p01, p02;
    double p10, p11, p12;
    double p20, p21, p22;
};

struct bezierqq1f_coeff_s
{
    /* coeffs at 0.0 - 0.5 - 1.0 */
    double c00, c01, c02;
    double c10, c11, c12;
    double c20, c21, c22;
};

CS2_API void bezierqq1f_from_qq(struct bezierqq1f_s *b, const struct bezierqq1f_coeff_s *c);
CS2_API double bezierqq1f_eval(const struct bezierqq1f_s *b, double u, double v);

CS2_API_END

#endif /* CS2_BEZIERQQ1F_H */
