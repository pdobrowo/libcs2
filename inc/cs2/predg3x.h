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
#ifndef CS2_PREDG3X_H
#define CS2_PREDG3X_H

#include "defs.h"
#include "vec3x.h"
/*#include "vec4f.h"*/
/*#include "mat44f.h"*/
#include "predh3x.h"
#include "preds3x.h"
/*#include "spin3x.h"*/
#include <gmp.h>

CS2_API_BEGIN

/**
 * general predicate:
 *
 *    (K x L) * rot(A - B) + (K - L) * rot(A x B) + c
 */
struct cs2_predg3x_s
{
    struct cs2_vec3x_s k, l, a, b;
    mpz_t c;
};

CS2_API void cs2_predg3x_init(struct cs2_predg3x_s *g);
CS2_API void cs2_predg3x_clear(struct cs2_predg3x_s *g);

CS2_API void cs2_predg3x_from_predh3x(struct cs2_predg3x_s *g, const struct cs2_predh3x_s *h);
CS2_API void cs2_predg3x_from_preds3x(struct cs2_predg3x_s *g, const struct cs2_preds3x_s *s);
CS2_API void cs2_predg3x_pquv(struct cs2_vec3x_s *p, struct cs2_vec3x_s *q, struct cs2_vec3x_s *u, struct cs2_vec3x_s *v, const struct cs2_predg3x_s *g);

/* type */
enum cs2_predgtype3x_e
{
    cs2_predgtype3x_improper,
    cs2_predgtype3x_ellipsoidal,
    cs2_predgtype3x_toroidal
};

CS2_API const char *cs2_predgtype3x_str(enum cs2_predgtype3x_e t);

CS2_API enum cs2_predgtype3x_e cs2_predg3x_type(const struct cs2_predg3x_s *g);

#if 0

/* parametrization */
struct predgparam3x_s
{
    struct mat44f_s q;
    double r12, r23, r31;
};

CS2_API void predg3x_param(struct predgparam3x_s *pp, const struct predg3x_s *g);
CS2_API void predgparam3x_eval(sstruct pin3x_s *s, const struct predgparam3x_s *pp, double u, double v, double sgn);

/* special */
CS2_API void predg3x_eigen(struct mat44f_s *m, struct vec4f_s *e, const struct predg3x_s *g);

#endif

CS2_API_END

#endif /* CS2_PREDG3X_H */
