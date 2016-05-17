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
#ifndef LIBCS2_VEC3X_H
#define LIBCS2_VEC3X_H

#include <gmp.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

struct vec3x_s
{
    mpz_t x, y, z;
};

typedef struct vec3x_s vec3x_t;

void vec3x_init(vec3x_t *v);
void vec3x_clear(vec3x_t *v);

void vec3x_set(vec3x_t *r, mpz_srcptr x, mpz_srcptr y, mpz_srcptr z);
void vec3x_set_si(vec3x_t *r, long x, long y, long z);
void vec3x_zero(vec3x_t *r);

int vec3x_is_zero(const vec3x_t *v);

void vec3x_copy(vec3x_t *r, const vec3x_t *v);

void vec3x_add(vec3x_t *r, const vec3x_t *a, const vec3x_t *b);
void vec3x_sub(vec3x_t *r, const vec3x_t *a, const vec3x_t *b);
void vec3x_neg(vec3x_t *r, const vec3x_t *v);
void vec3x_mul(vec3x_t *r, const vec3x_t *a, mpz_srcptr as);

void vec3x_mad2(vec3x_t *r, const vec3x_t *a, mpz_srcptr as, const vec3x_t *b, mpz_srcptr bs);
void vec3x_mad3(vec3x_t *r, const vec3x_t *a, mpz_srcptr as, const vec3x_t *b, mpz_srcptr bs, const vec3x_t *c, mpz_srcptr cs);
void vec3x_mad4(vec3x_t *r, const vec3x_t *a, mpz_srcptr as, const vec3x_t *b, mpz_srcptr bs, const vec3x_t *c, mpz_srcptr cs, const vec3x_t *d, mpz_srcptr ds);
void vec3x_mad5(vec3x_t *r, const vec3x_t *a, mpz_srcptr as, const vec3x_t *b, mpz_srcptr bs, const vec3x_t *c, mpz_srcptr cs, const vec3x_t *d, mpz_srcptr ds, const vec3x_t *e, mpz_srcptr es);

void vec3x_dot(mpz_ptr r, const vec3x_t *a, const vec3x_t *b);
void vec3x_cross(vec3x_t *r, const vec3x_t *a, const vec3x_t *b);

void vec3x_sqlen(mpz_ptr r, const vec3x_t *v);

void vec3x_tr(mpz_ptr r, const vec3x_t *v);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBCS2_VEC3X_H */
