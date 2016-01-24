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
#ifndef LIBCS2_SPINQUAD3X_H
#define LIBCS2_SPINQUAD3X_H

#include "predh3x.h"
#include "preds3x.h"
#include "predg3x.h"
#include "pin3x.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

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
struct spinquad3x_s
{
    mpz_t a11, a22, a33, a44, a12, a13, a14, a23, a24, a34;
};

typedef struct spinquad3x_s spinquad3x_t;

void spinquad3x_init(spinquad3x_t *sq);
void spinquad3x_clear(spinquad3x_t *sq);

void spinquad3x_from_predh3x(spinquad3x_t *sq, const predh3x_t *ph);
void spinquad3x_from_preds3x(spinquad3x_t *sq, const preds3x_t *ps);
void spinquad3x_from_predg3x(spinquad3x_t *sq, const predg3x_t *pg);

void spinquad3x_eval(mpz_t v, const spinquad3x_t *sq, const pin3x_t *p);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBCS2_SPINQUAD3X_H */