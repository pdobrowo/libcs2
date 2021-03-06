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
#ifndef CS2_HULL4F_H
#define CS2_HULL4F_H

#include "defs.h"
#include "vec4f.h"
#include "plane4f.h"
#include <stddef.h>
#include <stdio.h>

CS2_API_BEGIN

/**
 * convex hull in 4 dimensions
 *
 */
struct cs2_hull4f_s
{
    /* h-rep */
    struct cs2_plane4f_s *hr;
    size_t nhr;

    /* v-rep */
    struct cs2_vec4f_s *vr;
    size_t nvr;

    /* volume and area */
    double vol, area;
};

CS2_API void cs2_hull4f_init(struct cs2_hull4f_s *h);
CS2_API void cs2_hull4f_clear(struct cs2_hull4f_s *h);

CS2_API void cs2_hull4f_from_arr(struct cs2_hull4f_s *h, const struct cs2_vec4f_s *v, size_t n);
CS2_API int cs2_hull4f_inter(const struct cs2_hull4f_s *ha, const struct cs2_hull4f_s *hb);

CS2_API void cs2_hull4f_print_json(struct cs2_hull4f_s *h, FILE *f, size_t indent);

CS2_API_END

#endif /* CS2_HULL4F_H */
