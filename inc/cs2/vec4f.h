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
#ifndef LIBCS2_VEC4F_H
#define LIBCS2_VEC4F_H

#include "pin3f.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

struct vec4f_s
{
    double x, y, z, w;
};

typedef struct vec4f_s vec4f_t;

void vec4f_set(vec4f_t *r, double x, double y, double z, double w);
void vec4f_zero(vec4f_t *r);

void vec4f_add(vec4f_t *r, const vec4f_t *a, const vec4f_t *b);
void vec4f_sub(vec4f_t *r, const vec4f_t *a, const vec4f_t *b);
void vec4f_neg(vec4f_t *r, const vec4f_t *v);
void vec4f_mul(vec4f_t *r, const vec4f_t *a, double as);

void vec4f_mad2(vec4f_t *r, const vec4f_t *a, double as, const vec4f_t *b, double bs);
void vec4f_mad3(vec4f_t *r, const vec4f_t *a, double as, const vec4f_t *b, double bs, const vec4f_t *c, double cs);
void vec4f_mad4(vec4f_t *r, const vec4f_t *a, double as, const vec4f_t *b, double bs, const vec4f_t *c, double cs, const vec4f_t *d, double ds);
void vec4f_mad5(vec4f_t *r, const vec4f_t *a, double as, const vec4f_t *b, double bs, const vec4f_t *c, double cs, const vec4f_t *d, double ds, const vec4f_t *e, double es);

double vec4f_dot(const vec4f_t *a, const vec4f_t *b);

double vec4f_len(const vec4f_t *v);
double vec4f_sqlen(const vec4f_t *v);

double vec4f_tr(const vec4f_t *v);

void vec4f_from_pin3f(vec4f_t *r, const pin3f_t *p);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBCS2_VEC4F_H */
