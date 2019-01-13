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
#ifndef CS2_BEZIERQQ4F_H
#define CS2_BEZIERQQ4F_H

#include "defs.h"
#include "vec4f.h"
#include "hull4f.h"

CS2_API_BEGIN

/**
 * bezier quadratic-quadratic patch in 4 dimensions
 *
 * index (uv):
 *
 * 00 01 02
 * 10 11 12
 * 20 21 22
 */
struct cs2_bezierqq4f_s
{
    /* control */
    struct cs2_vec4f_s p00, p01, p02;
    struct cs2_vec4f_s p10, p11, p12;
    struct cs2_vec4f_s p20, p21, p22;

    /* hull */
    struct cs2_hull4f_s h;
};

struct cs2_bezierqq4f_coeff_s
{
    /* coeffs at 0.0 - 0.5 - 1.0 */
    struct cs2_vec4f_s c00, c01, c02;
    struct cs2_vec4f_s c10, c11, c12;
    struct cs2_vec4f_s c20, c21, c22;
};

CS2_API void cs2_bezierqq4f_init(struct cs2_bezierqq4f_s *b);
CS2_API void cs2_bezierqq4f_clear(struct cs2_bezierqq4f_s *b);

CS2_API void cs2_bezierqq4f_from_qq(struct cs2_bezierqq4f_s *b, const struct cs2_bezierqq4f_coeff_s *c);
CS2_API void cs2_bezierqq4f_eval(struct cs2_vec4f_s *r, const struct cs2_bezierqq4f_s *b, double u, double v);
CS2_API int cs2_bezierqq4f_inter(const struct cs2_bezierqq4f_s *p, const struct cs2_bezierqq4f_s *q);

CS2_API_END

#endif /* CS2_BEZIERQQ4F_H */
