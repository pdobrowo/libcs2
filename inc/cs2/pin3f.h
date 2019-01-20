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

CS2_API void cs2_pin3f_set(struct cs2_pin3f_s *r, double p12, double p23, double p31, double p0);
CS2_API void cs2_pin3f_zero(struct cs2_pin3f_s *r);

CS2_API void cs2_pin3f_add(struct cs2_pin3f_s *r, const struct cs2_pin3f_s *a, const struct cs2_pin3f_s *b);
CS2_API void cs2_pin3f_sub(struct cs2_pin3f_s *r, const struct cs2_pin3f_s *a, const struct cs2_pin3f_s *b);
CS2_API void cs2_pin3f_neg(struct cs2_pin3f_s *r, const struct cs2_pin3f_s *v);
CS2_API void cs2_pin3f_mul(struct cs2_pin3f_s *r, const struct cs2_pin3f_s *a, double as);

CS2_API void cs2_pin3f_cl(struct cs2_pin3f_s *r, const struct cs2_vec3f_s *a, const struct cs2_vec3f_s *b);

CS2_API void cs2_pin3f_mad2(struct cs2_pin3f_s *r, const struct cs2_pin3f_s *a, double as, const struct cs2_pin3f_s *b, double bs);
CS2_API void cs2_pin3f_mad3(struct cs2_pin3f_s *r, const struct cs2_pin3f_s *a, double as, const struct cs2_pin3f_s *b, double bs, const struct cs2_pin3f_s *c, double cs);
CS2_API void cs2_pin3f_mad4(struct cs2_pin3f_s *r, const struct cs2_pin3f_s *a, double as, const struct cs2_pin3f_s *b, double bs, const struct cs2_pin3f_s *c, double cs, const struct cs2_pin3f_s *d, double ds);
CS2_API void cs2_pin3f_mad5(struct cs2_pin3f_s *r, const struct cs2_pin3f_s *a, double as, const struct cs2_pin3f_s *b, double bs, const struct cs2_pin3f_s *c, double cs, const struct cs2_pin3f_s *d, double ds, const struct cs2_pin3f_s *e, double es);

CS2_API double cs2_pin3f_len(const struct cs2_pin3f_s *v);
CS2_API double cs2_pin3f_sqlen(const struct cs2_pin3f_s *v);

CS2_API_END

#endif /* CS2_PIN3F_H */
