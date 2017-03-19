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
#ifndef CS2_PREDTT3F_H
#define CS2_PREDTT3F_H

#include "defs.h"
#include "preds3f.h"
#include "vec3f.h"

CS2_API_BEGIN

/**
 * triangle-triangle predicate:
 *
 *    klm - a stationary triangle
 *    abc - a rotating triangle
 */
struct predtt3f_s
{
    struct vec3f_s k, l, m;
    struct vec3f_s a, b, c;
};

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
    struct preds3f_s s[3][3];
};

CS2_API void predtt3f_decomp(struct predttdecomp3f_s *d, const struct predtt3f_s *tt);

CS2_API_END

#endif /* CS2_PREDTT3F_H */
