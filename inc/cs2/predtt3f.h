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
#ifndef LIBCS2_PREDTT3F_H
#define LIBCS2_PREDTT3F_H

#include "preds3f.h"
#include "vec3f.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * triangle-triangle predicate:
 *
 *    klm - a stationary triangle
 *    abc - a rotating triangle
 */
struct predtt3f_s
{
    vec3f_t k, l, m;
    vec3f_t a, b, c;
};

typedef struct predtt3f_s predtt3f_t;

/**
 * decomposition
 *
 * index:
 * i = { 'kl', 'lm', 'mk' }
 * j = { 'ab', 'bc', 'ca' }
 *
 * [kl/ab] [kl/bc] [kl/ca]
 * [lm/ab] [lm/bc] [lm/ca]
 * [mk/ab] [mk/bc] [mk/ca]
 *
 */
struct predttdecomp3f_s
{
    preds3f_t s[3][3];
};

typedef struct predttdecomp3f_s predttdecomp3f_t;

void predtt3f_decomp(predttdecomp3f_t *d, const predtt3f_t *tt);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBCS2_PREDTT3F_H */
