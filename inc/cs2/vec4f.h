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

CS2_API void cs2_vec4f_set(struct cs2_vec4f_s *v, double x, double y, double z, double w);
CS2_API void cs2_vec4f_zero(struct cs2_vec4f_s *v);

CS2_API double cs2_vec4f_coord(const struct cs2_vec4f_s *v, int i);

CS2_API void cs2_vec4f_copy(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va);

CS2_API void cs2_vec4f_add(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va, const struct cs2_vec4f_s *vb);
CS2_API void cs2_vec4f_sub(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va, const struct cs2_vec4f_s *vb);
CS2_API void cs2_vec4f_neg(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va);
CS2_API void cs2_vec4f_mul(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va, double sa);

CS2_API void cs2_vec4f_mad2(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va, double sa, const struct cs2_vec4f_s *vb, double sb);
CS2_API void cs2_vec4f_mad3(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va, double sa, const struct cs2_vec4f_s *vb, double sb, const struct cs2_vec4f_s *vc, double sc);
CS2_API void cs2_vec4f_mad4(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va, double sa, const struct cs2_vec4f_s *vb, double sb, const struct cs2_vec4f_s *vc, double sc, const struct cs2_vec4f_s *vd, double sd);
CS2_API void cs2_vec4f_mad5(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va, double sa, const struct cs2_vec4f_s *vb, double sb, const struct cs2_vec4f_s *vc, double sc, const struct cs2_vec4f_s *vd, double sd, const struct cs2_vec4f_s *ve, double se);

CS2_API double cs2_vec4f_dot(const struct cs2_vec4f_s *va, const struct cs2_vec4f_s *vb);

CS2_API void cs2_vec4f_unit(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va);

CS2_API double cs2_vec4f_len(const struct cs2_vec4f_s *v);
CS2_API double cs2_vec4f_sqlen(const struct cs2_vec4f_s *v);

CS2_API double cs2_vec4f_tr(const struct cs2_vec4f_s *v);

CS2_API void cs2_vec4f_from_pin3f(struct cs2_vec4f_s *v, const struct cs2_pin3f_s *p);

CS2_API void cs2_vec4f_print_json(const struct cs2_vec4f_s *v, FILE *f, size_t indent);

CS2_API_END

#endif /* CS2_VEC4F_H */
