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
#ifndef CS2_SPINQUAD3X_H
#define CS2_SPINQUAD3X_H

#include "defs.h"
#include "predh3x.h"
#include "preds3x.h"
#include "predg3x.h"
#include "pin3x.h"

CS2_API_BEGIN

/**
 * spin quadric:
 *
 *    F(s) = a11 * s12^2 + a22 * s23^2 + a33 * s31^2 + a44 * s0^2
 *           + 2 (a12 * s12 * s23 + a13 * s12 * s31 + a14 * s12 * s0
 *              + a23 * s23 * s31 + a24 * s23 * s0 + a34 * s31 * s0)
 *
 *          | a11 a12 a13 a14 |
 *    Q_s = | a12 a22 a23 a24 |
 *          | a13 a23 a33 a34 |
 *          | a14 a24 a34 a44 |
 *
 *    s = [s12; s23, s31; s0]^T
 *
 *    F(s) = s^T Q_s s
 *
 *    s12^2 + s23^2 + s31^2 + s0^2 = 1
 */
struct cs2_spinquad3x_s
{
    mpz_t a11, a22, a33, a44, a12, a13, a14, a23, a24, a34;
};

CS2_API void cs2_spinquad3x_init(struct cs2_spinquad3x_s *sq);
CS2_API void cs2_spinquad3x_clear(struct cs2_spinquad3x_s *sq);

CS2_API void cs2_spinquad3x_from_predh3x(struct cs2_spinquad3x_s *sq, const struct cs2_predh3x_s *ph);
CS2_API void cs2_spinquad3x_from_preds3x(struct cs2_spinquad3x_s *sq, const struct cs2_preds3x_s *ps);
CS2_API void cs2_spinquad3x_from_predg3x(struct cs2_spinquad3x_s *sq, const struct cs2_predg3x_s *pg);

CS2_API void cs2_spinquad3x_eval(mpz_ptr s, const struct cs2_spinquad3x_s *sq, const struct cs2_pin3x_s *p);

CS2_API_END

#endif /* CS2_SPINQUAD3X_H */
