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
#ifndef CS2_PIN3F_H
#define CS2_PIN3F_H

#include "defs.h"
#include "vec3f.h"
#include <gmp.h>

CS2_API_BEGIN

struct cs2_pin3f_s
{
    double p12, p23, p31, p0;
};

CS2_API const struct cs2_pin3f_s CS2_PIN3F_ZERO;
CS2_API const struct cs2_pin3f_s CS2_PIN3F_ONE;

CS2_API void cs2_pin3f_set(struct cs2_pin3f_s *p, double p12, double p23, double p31, double p0);
CS2_API void cs2_pin3f_zero(struct cs2_pin3f_s *p);

CS2_API void cs2_pin3f_add(struct cs2_pin3f_s *p, const struct cs2_pin3f_s *pa, const struct cs2_pin3f_s *pb);
CS2_API void cs2_pin3f_sub(struct cs2_pin3f_s *p, const struct cs2_pin3f_s *pa, const struct cs2_pin3f_s *pb);
CS2_API void cs2_pin3f_neg(struct cs2_pin3f_s *p, const struct cs2_pin3f_s *pa);
CS2_API void cs2_pin3f_mul(struct cs2_pin3f_s *p, const struct cs2_pin3f_s *pa, double sa);

CS2_API void cs2_pin3f_cl(struct cs2_pin3f_s *p, const struct cs2_vec3f_s *va, const struct cs2_vec3f_s *vb);

CS2_API void cs2_pin3f_mad2(struct cs2_pin3f_s *p, const struct cs2_pin3f_s *pa, double sa, const struct cs2_pin3f_s *pb, double sb);
CS2_API void cs2_pin3f_mad3(struct cs2_pin3f_s *p, const struct cs2_pin3f_s *pa, double sa, const struct cs2_pin3f_s *pb, double sb, const struct cs2_pin3f_s *pc, double sc);
CS2_API void cs2_pin3f_mad4(struct cs2_pin3f_s *p, const struct cs2_pin3f_s *pa, double sa, const struct cs2_pin3f_s *pb, double sb, const struct cs2_pin3f_s *pc, double sc, const struct cs2_pin3f_s *vd, double sd);
CS2_API void cs2_pin3f_mad5(struct cs2_pin3f_s *p, const struct cs2_pin3f_s *pa, double sa, const struct cs2_pin3f_s *pb, double sb, const struct cs2_pin3f_s *pc, double sc, const struct cs2_pin3f_s *vd, double sd, const struct cs2_pin3f_s *ve, double se);

CS2_API double cs2_pin3f_len(const struct cs2_pin3f_s *p);
CS2_API double cs2_pin3f_sqlen(const struct cs2_pin3f_s *p);

CS2_API_END

#endif /* CS2_PIN3F_H */
