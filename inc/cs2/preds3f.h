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
#ifndef CS2_PREDS3F_H
#define CS2_PREDS3F_H

#include "defs.h"
#include "vec3f.h"

CS2_API_BEGIN

/**
 * screw predicate:
 *
 *    (K x L) * rot(A - B) + (K - L) * rot(A x B)
 */
struct cs2_preds3f_s
{
    struct cs2_vec3f_s k, l, a, b;
};

CS2_API void cs2_preds3f_set(struct cs2_preds3f_s *ps, const struct cs2_vec3f_s *vk, const struct cs2_vec3f_s *vl, const struct cs2_vec3f_s *va, const struct cs2_vec3f_s *vb);
CS2_API void cs2_preds3f_copy(struct cs2_preds3f_s *ps, const struct cs2_preds3f_s *psa);

CS2_API_END

#endif /* CS2_PREDS3F_H */
