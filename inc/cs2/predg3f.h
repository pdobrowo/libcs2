/**
 * Copyright (c) 2015 Przemysław Dobrowolski
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
#ifndef LIBCS2_PREDG3F_H
#define LIBCS2_PREDG3F_H

#include "vec3f.h"
#include "predh3f.h"
#include "preds3f.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * general predicate:
 *
 *    (K x L) * rot(A - B) + (K - L) * rot(A x B) + c
 */
struct predg3f_s
{
    vec3f_t k, l, a, b;
    double c;
};

typedef struct predg3f_s predg3f_t;

void predg3f_from_predh3f(predg3f_t *g, const predh3f_t *h);
void predg3f_from_preds3f(predg3f_t *g, const preds3f_t *s);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBCS2_PREDG3F_H */
