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
#ifndef CS2_PLANE3F_H
#define CS2_PLANE3F_H

#include "defs.h"
#include "vec3f.h"
#include <stddef.h>
#include <stdio.h>

CS2_API_BEGIN

struct plane3f_s
{
    struct vec3f_s n;
    double d;
};

CS2_API void plane3f_set(struct plane3f_s *p, const struct vec3f_s *n, double d);
CS2_API void plane3f_copy(struct plane3f_s *r, const struct plane3f_s *p);
CS2_API double plane3f_pops(const struct plane3f_s *r, const struct vec3f_s *p);

CS2_API void plane3f_print_json(const struct plane3f_s *r, FILE *f, size_t ind);

CS2_API_END

#endif /* CS2_PLANE3F_H */
