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
#ifndef CS2_PREDG3F_H
#define CS2_PREDG3F_H

#include "defs.h"
#include "vec3f.h"
#include "vec4f.h"
#include "mat44f.h"
#include "predh3f.h"
#include "preds3f.h"
#include "spin3f.h"

CS2_API_BEGIN

/**
 * general predicate:
 *
 *    (K x L) * rot(A - B) + (K - L) * rot(A x B) + c
 */
struct predg3f_s
{
    struct vec3f_s k, l, a, b;
    double c;
};

CS2_API void predg3f_set(struct predg3f_s *g, const struct vec3f_s *k, const struct vec3f_s *l, const struct vec3f_s *a, const struct vec3f_s *b, double c);
CS2_API void predg3f_copy(struct predg3f_s *r, const struct predg3f_s *g);

CS2_API void predg3f_from_predh3f(struct predg3f_s *g, const struct predh3f_s *h);
CS2_API void predg3f_from_preds3f(struct predg3f_s *g, const struct preds3f_s *s);
CS2_API void predg3f_pquv(struct vec3f_s *p, struct vec3f_s *q, struct vec3f_s *u, struct vec3f_s *v, const struct predg3f_s *g);

/* type */
enum predgtype3f_e
{
    predgtype3f_inproper,
    predgtype3f_ellipsoidal,
    predgtype3f_toroidal
};

CS2_API const char *predgtype3f_str(enum predgtype3f_e t);

CS2_API enum predgtype3f_e predg3f_type(const struct predg3f_s *g);

/* parametrization */
enum predgparamtype3f_e
{
    /* common */
    predgparamtype3f_an_empty_set,

    /* ellipsoidal */
    predgparamtype3f_a_pair_of_points,
    predgparamtype3f_a_pair_of_separate_ellipsoids,
    predgparamtype3f_a_pair_of_y_touching_ellipsoids,
    predgparamtype3f_a_pair_of_yz_crossed_ellipsoids,
    predgparamtype3f_a_pair_of_z_touching_ellipsoids,
    predgparamtype3f_a_y_barrel,
    predgparamtype3f_a_z_barrel,
    predgparamtype3f_a_notched_y_barrel,
    predgparamtype3f_a_notched_z_barrel,
    predgparamtype3f_a_pair_of_separate_yz_caps,

    /* toroidal */
    predgparamtype3f_a_torus
};

CS2_API const char *predgparamtype3f_str(enum predgparamtype3f_e pt);

CS2_API int predgparamtype3f_dim(enum predgparamtype3f_e pt);
CS2_API int predgparamtype3f_components(enum predgparamtype3f_e pt);

struct predgparam3f_s
{
    enum predgparamtype3f_e t;
    struct mat44f_s q;
    double a, b, c;
};

CS2_API void predg3f_param(struct predgparam3f_s *pp, const struct predg3f_s *g);
CS2_API void predgparam3f_eval(struct spin3f_s *s, const struct predgparam3f_s *pp, double u, double v, int component);

/* special */
CS2_API void predg3f_eigen(struct mat44f_s *m, struct vec4f_s *e, const struct predg3f_s *g);

CS2_API_END

#endif /* CS2_PREDG3F_H */
