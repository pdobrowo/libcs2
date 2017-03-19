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
#ifndef CS2_VEC3F_H
#define CS2_VEC3F_H

#include "defs.h"
#include "pin3f.h"
#include <stddef.h>
#include <stdio.h>

CS2_API_BEGIN

struct vec3f_s
{
    double x, y, z;
};

CS2_API void vec3f_set(struct vec3f_s *r, double x, double y, double z);
CS2_API void vec3f_zero(struct vec3f_s *r);

CS2_API void vec3f_copy(struct vec3f_s *r, const struct vec3f_s *v);

CS2_API void vec3f_add(struct vec3f_s *r, const struct vec3f_s *a, const struct vec3f_s *b);
CS2_API void vec3f_sub(struct vec3f_s *r, const struct vec3f_s *a, const struct vec3f_s *b);
CS2_API void vec3f_neg(struct vec3f_s *r, const struct vec3f_s *v);
CS2_API void vec3f_mul(struct vec3f_s *r, const struct vec3f_s *a, double as);

CS2_API void vec3f_cl(struct pin3f_s *r, const struct vec3f_s *a, const struct vec3f_s *b);

CS2_API void vec3f_mad2(struct vec3f_s *r, const struct vec3f_s *a, double as, const struct vec3f_s *b, double bs);
CS2_API void vec3f_mad3(struct vec3f_s *r, const struct vec3f_s *a, double as, const struct vec3f_s *b, double bs, const struct vec3f_s *c, double cs);
CS2_API void vec3f_mad4(struct vec3f_s *r, const struct vec3f_s *a, double as, const struct vec3f_s *b, double bs, const struct vec3f_s *c, double cs, const struct vec3f_s *d, double ds);
CS2_API void vec3f_mad5(struct vec3f_s *r, const struct vec3f_s *a, double as, const struct vec3f_s *b, double bs, const struct vec3f_s *c, double cs, const struct vec3f_s *d, double ds, const struct vec3f_s *e, double es);

CS2_API double vec3f_dot(const struct vec3f_s *a, const struct vec3f_s *b);
CS2_API void vec3f_cross(struct vec3f_s *r, const struct vec3f_s *a, const struct vec3f_s *b);

CS2_API void vec3f_unit(struct vec3f_s *r, const struct vec3f_s *v);

CS2_API double vec3f_len(const struct vec3f_s *v);
CS2_API double vec3f_sqlen(const struct vec3f_s *v);

CS2_API double vec3f_tr(const struct vec3f_s *v);

CS2_API void vec3f_print_json(const struct vec3f_s *r, FILE *f, size_t ind);

CS2_API_END

#endif /* CS2_VEC3F_H */
