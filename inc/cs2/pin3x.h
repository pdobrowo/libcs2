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
#ifndef LIBCS2_PIN3X_H
#define LIBCS2_PIN3X_H

#include <gmp.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

struct pin3x_s
{
    mpz_t p12, p23, p31, p0;
};

typedef struct pin3x_s pin3x_t;

void pin3x_init(pin3x_t *p);
void pin3x_clear(pin3x_t *p);

void pin3x_set(pin3x_t *p, mpz_srcptr p12, mpz_srcptr p23, mpz_srcptr p31, mpz_srcptr p0);
void pin3x_set_si(pin3x_t *p, long p12, long p23, long p31, long p0);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBCS2_PIN3X_H */
