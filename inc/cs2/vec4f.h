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
#ifndef CS2_VEC4F_H
#define CS2_VEC4F_H

#include "defs.h"
#include "pin3f.h"
#include <stddef.h>
#include <stdio.h>

CS2_API_BEGIN

struct cs2_vec4f_s
{
    double x, y, z, w;
};

CS2_API void cs2_vec4f_set(struct cs2_vec4f_s *r, double x, double y, double z, double w);
CS2_API void cs2_vec4f_zero(struct cs2_vec4f_s *r);

CS2_API void cs2_vec4f_copy(struct cs2_vec4f_s *r, const struct cs2_vec4f_s *v);

CS2_API void cs2_vec4f_add(struct cs2_vec4f_s *r, const struct cs2_vec4f_s *a, const struct cs2_vec4f_s *b);
CS2_API void cs2_vec4f_sub(struct cs2_vec4f_s *r, const struct cs2_vec4f_s *a, const struct cs2_vec4f_s *b);
CS2_API void cs2_vec4f_neg(struct cs2_vec4f_s *r, const struct cs2_vec4f_s *v);
CS2_API void cs2_vec4f_mul(struct cs2_vec4f_s *r, const struct cs2_vec4f_s *a, double as);

CS2_API void cs2_vec4f_mad2(struct cs2_vec4f_s *r, const struct cs2_vec4f_s *a, double as, const struct cs2_vec4f_s *b, double bs);
CS2_API void cs2_vec4f_mad3(struct cs2_vec4f_s *r, const struct cs2_vec4f_s *a, double as, const struct cs2_vec4f_s *b, double bs, const struct cs2_vec4f_s *c, double cs);
CS2_API void cs2_vec4f_mad4(struct cs2_vec4f_s *r, const struct cs2_vec4f_s *a, double as, const struct cs2_vec4f_s *b, double bs, const struct cs2_vec4f_s *c, double cs, const struct cs2_vec4f_s *d, double ds);
CS2_API void cs2_vec4f_mad5(struct cs2_vec4f_s *r, const struct cs2_vec4f_s *a, double as, const struct cs2_vec4f_s *b, double bs, const struct cs2_vec4f_s *c, double cs, const struct cs2_vec4f_s *d, double ds, const struct cs2_vec4f_s *e, double es);

CS2_API double cs2_vec4f_dot(const struct cs2_vec4f_s *a, const struct cs2_vec4f_s *b);

CS2_API double cs2_vec4f_len(const struct cs2_vec4f_s *v);
CS2_API double cs2_vec4f_sqlen(const struct cs2_vec4f_s *v);

CS2_API double cs2_vec4f_tr(const struct cs2_vec4f_s *v);

CS2_API void cs2_vec4f_from_pin3f(struct cs2_vec4f_s *r, const struct cs2_pin3f_s *p);

CS2_API void cs2_vec4f_print_json(const struct cs2_vec4f_s *r, FILE *f, size_t ind);

CS2_API_END

#endif /* CS2_VEC4F_H */
