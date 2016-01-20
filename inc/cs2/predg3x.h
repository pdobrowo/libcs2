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
#ifndef LIBCS2_PREDG3X_H
#define LIBCS2_PREDG3X_H

#include "vec3x.h"
/*#include "vec4f.h"*/
/*#include "mat44f.h"*/
#include "predh3x.h"
#include "preds3x.h"
/*#include "spin3x.h"*/
#include <gmp.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * general predicate:
 *
 *    (K x L) * rot(A - B) + (K - L) * rot(A x B) + c
 */
struct predg3x_s
{
    vec3x_t k, l, a, b;
    mpz_t c;
};

typedef struct predg3x_s predg3x_t;

void predg3x_init(predg3x_t *g);
void predg3x_clear(predg3x_t *g);

void predg3x_from_predh3x(predg3x_t *g, const predh3x_t *h);
void predg3x_from_preds3x(predg3x_t *g, const preds3x_t *s);
void predg3x_pquv(vec3x_t *p, vec3x_t *q, vec3x_t *u, vec3x_t *v, const predg3x_t *g);

/* type */
enum predgtype3x_e
{
    predgtype3x_inproper,
    predgtype3x_ellipsoidal,
    predgtype3x_toroidal
};

typedef enum predgtype3x_e predgtype3x_t;

const char *predgtype3x_str(predgtype3x_t t);

predgtype3x_t predg3x_type(const predg3x_t *g);

#if 0

/* parametrization */
struct predgparam3x_s
{
    mat44f_t q;
    double r12, r23, r31;
};

typedef struct predgparam3x_s predgparam3x_t;

void predg3x_param(predgparam3x_t *pp, const predg3x_t *g);
void predgparam3x_eval(spin3x_t *s, const predgparam3x_t *pp, double u, double v, double sgn);

/* special */
void predg3x_eigen(mat44f_t *m, vec4f_t *e, const predg3x_t *g);

#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBCS2_PREDG3X_H */
