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
#ifndef CS2_RAND_H
#define CS2_RAND_H

#include "defs.h"
#include "vec3f.h"
#include <stdint.h>

CS2_API_BEGIN

struct cs2_rand_s
{
    uint64_t state[2];
};

CS2_API void cs2_rand_seed(struct cs2_rand_s *r);

CS2_API double cs2_rand_1f(struct cs2_rand_s *r); /* uniform rand [0; 1] */
CS2_API double cs2_rand_u1f(struct cs2_rand_s *r, double min, double max); /* uniform rand [min; max] */

CS2_API int cs2_rand_1i(struct cs2_rand_s *r); /* uniform rand {0, 1} */
CS2_API int cs2_rand_u1i(struct cs2_rand_s *r, int min, int max); /* uniform rand {min, ..., max} */

CS2_API void cs2_rand_vec3f_1f(struct cs2_vec3f_s *v, struct cs2_rand_s *r); /* uniform rand vec3f [0; 1] */
CS2_API void cs2_rand_vec3f_u1f(struct cs2_vec3f_s *v, struct cs2_rand_s *r, double min, double max); /* uniform rand vec3f [min; max] */

CS2_API_END

#endif /* CS2_RAND_H */
