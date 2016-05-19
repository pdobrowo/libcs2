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
#ifndef CS2_PIN3F_H
#define CS2_PIN3F_H

#include "defs.h"
#include <gmp.h>

CS2_API_BEGIN

struct pin3f_s
{
    double p12, p23, p31, p0;
};

typedef struct pin3f_s pin3f_t;

CS2_API const pin3f_t PIN3F_ZERO;
CS2_API const pin3f_t PIN3F_ONE;

CS2_API void pin3f_set(pin3f_t *r, double p12, double p23, double p31, double p0);
CS2_API void pin3f_zero(pin3f_t *r);

CS2_API void pin3f_add(pin3f_t *r, const pin3f_t *a, const pin3f_t *b);
CS2_API void pin3f_sub(pin3f_t *r, const pin3f_t *a, const pin3f_t *b);
CS2_API void pin3f_neg(pin3f_t *r, const pin3f_t *v);
CS2_API void pin3f_mul(pin3f_t *r, const pin3f_t *a, double as);

CS2_API void pin3f_mad2(pin3f_t *r, const pin3f_t *a, double as, const pin3f_t *b, double bs);
CS2_API void pin3f_mad3(pin3f_t *r, const pin3f_t *a, double as, const pin3f_t *b, double bs, const pin3f_t *c, double cs);
CS2_API void pin3f_mad4(pin3f_t *r, const pin3f_t *a, double as, const pin3f_t *b, double bs, const pin3f_t *c, double cs, const pin3f_t *d, double ds);
CS2_API void pin3f_mad5(pin3f_t *r, const pin3f_t *a, double as, const pin3f_t *b, double bs, const pin3f_t *c, double cs, const pin3f_t *d, double ds, const pin3f_t *e, double es);

CS2_API double pin3f_len(const pin3f_t *v);
CS2_API double pin3f_sqlen(const pin3f_t *v);

CS2_API_END

#endif /* CS2_PIN3F_H */
