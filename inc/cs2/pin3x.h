/**
 * Copyright (c) 2015-2017 Przemysław Dobrowolski
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
#ifndef CS2_PIN3X_H
#define CS2_PIN3X_H

#include "defs.h"
#include <gmp.h>

CS2_API_BEGIN

struct cs2_pin3x_s
{
    mpz_t p12, p23, p31, p0;
};

CS2_API void cs2_pin3x_init(struct cs2_pin3x_s *p);
CS2_API void cs2_pin3x_clear(struct cs2_pin3x_s *p);

CS2_API void cs2_pin3x_set(struct cs2_pin3x_s *p, mpz_srcptr p12, mpz_srcptr p23, mpz_srcptr p31, mpz_srcptr p0);
CS2_API void cs2_pin3x_set_si(struct cs2_pin3x_s *p, long p12, long p23, long p31, long p0);

CS2_API_END

#endif /* CS2_PIN3X_H */
