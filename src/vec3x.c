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
#include "cs2/vec3x.h"

void vec3x_init(vec3x_t *v)
{
    mpz_init(v->x);
    mpz_init(v->y);
    mpz_init(v->z);
}

void vec3x_clear(vec3x_t *v)
{
    mpz_clear(v->x);
    mpz_clear(v->y);
    mpz_clear(v->z);
}

void vec3x_set_si(vec3x_t *v, long x, long y, long z)
{
    mpz_set_si(v->x, x);
    mpz_set_si(v->y, y);
    mpz_set_si(v->z, z);
}

void vec3x_add(vec3x_t *r, const vec3x_t *a, const vec3x_t *b)
{
    mpz_add(r->x, a->x, b->x);
    mpz_add(r->y, a->y, b->y);
    mpz_add(r->z, a->z, b->z);
}
