/**
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Przemysław Dobrowolski
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

struct vec3x_s
{
    mpz_t x, y, z;
};

typedef struct vec3x_s vec3x_t;

void vec3x_init(vec3x_t *v);
void vec3x_clear(vec3x_t *v);

void vec3x_set_si(vec3x_t *v, long x, long y, long z);

void vec3x_add(vec3x_t *r, const vec3x_t *a, const vec3x_t *b);

#endif /* LIBCS2_VEC3X_H */
