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
#ifndef CS2_VEC3X_H
#define CS2_VEC3X_H

#include "defs.h"
#include <gmp.h>

CS2_API_BEGIN

struct vec3x_s
{
    mpz_t x, y, z;
};

CS2_API void vec3x_init(struct vec3x_s *v);
CS2_API void vec3x_clear(struct vec3x_s *v);

CS2_API void vec3x_set(struct vec3x_s *r, mpz_srcptr x, mpz_srcptr y, mpz_srcptr z);
CS2_API void vec3x_set_si(struct vec3x_s *r, long x, long y, long z);
CS2_API void vec3x_zero(struct vec3x_s *r);

CS2_API int vec3x_is_zero(const struct vec3x_s *v);

CS2_API void vec3x_copy(struct vec3x_s *r, const struct vec3x_s *v);

CS2_API void vec3x_add(struct vec3x_s *r, const struct vec3x_s *a, const struct vec3x_s *b);
CS2_API void vec3x_sub(struct vec3x_s *r, const struct vec3x_s *a, const struct vec3x_s *b);
CS2_API void vec3x_neg(struct vec3x_s *r, const struct vec3x_s *v);
CS2_API void vec3x_mul(struct vec3x_s *r, const struct vec3x_s *a, mpz_srcptr as);

CS2_API void vec3x_mad2(struct vec3x_s *r, const struct vec3x_s *a, mpz_srcptr as, const struct vec3x_s *b, mpz_srcptr bs);
CS2_API void vec3x_mad3(struct vec3x_s *r, const struct vec3x_s *a, mpz_srcptr as, const struct vec3x_s *b, mpz_srcptr bs, const struct vec3x_s *c, mpz_srcptr cs);
CS2_API void vec3x_mad4(struct vec3x_s *r, const struct vec3x_s *a, mpz_srcptr as, const struct vec3x_s *b, mpz_srcptr bs, const struct vec3x_s *c, mpz_srcptr cs, const struct vec3x_s *d, mpz_srcptr ds);
CS2_API void vec3x_mad5(struct vec3x_s *r, const struct vec3x_s *a, mpz_srcptr as, const struct vec3x_s *b, mpz_srcptr bs, const struct vec3x_s *c, mpz_srcptr cs, const struct vec3x_s *d, mpz_srcptr ds, const struct vec3x_s *e, mpz_srcptr es);

CS2_API void vec3x_dot(mpz_ptr r, const struct vec3x_s *a, const struct vec3x_s *b);
CS2_API void vec3x_cross(struct vec3x_s *r, const struct vec3x_s *a, const struct vec3x_s *b);

CS2_API void vec3x_sqlen(mpz_ptr r, const struct vec3x_s *v);

CS2_API void vec3x_tr(mpz_ptr r, const struct vec3x_s *v);

CS2_API_END

#endif /* CS2_VEC3X_H */
