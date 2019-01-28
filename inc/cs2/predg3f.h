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
struct cs2_predg3f_s
{
    struct cs2_vec3f_s k, l, a, b;
    double c;
};

CS2_API void cs2_predg3f_set(struct cs2_predg3f_s *g, const struct cs2_vec3f_s *k, const struct cs2_vec3f_s *l, const struct cs2_vec3f_s *a, const struct cs2_vec3f_s *b, double c);
CS2_API void cs2_predg3f_copy(struct cs2_predg3f_s *r, const struct cs2_predg3f_s *g);

CS2_API void cs2_predg3f_from_predh3f(struct cs2_predg3f_s *g, const struct cs2_predh3f_s *h);
CS2_API void cs2_predg3f_from_preds3f(struct cs2_predg3f_s *g, const struct cs2_preds3f_s *s);
CS2_API void cs2_predg3f_pquv(struct cs2_vec3f_s *p, struct cs2_vec3f_s *q, struct cs2_vec3f_s *u, struct cs2_vec3f_s *v, const struct cs2_predg3f_s *g);

/* type */
enum cs2_predgtype3f_e
{
    cs2_predgtype3f_improper,
    cs2_predgtype3f_ellipsoidal,
    cs2_predgtype3f_toroidal,

    cs2_predgtype3f_COUNT
};

CS2_API const char *cs2_predgtype3f_str(enum cs2_predgtype3f_e t);

CS2_API enum cs2_predgtype3f_e cs2_predg3f_type(const struct cs2_predg3f_s *g);

/**
 * parametrization (17 cases):
 * - empty set (1 case)
 * - ellipsoidal (10 cases)
 * - toroidal (6 cases)
 */
enum cs2_predgparamtype3f_e
{
    /* common */
    cs2_predgparamtype3f_an_empty_set,

    /* ellipsoidal */
    cs2_predgparamtype3f_a_pair_of_points,
    cs2_predgparamtype3f_a_pair_of_separate_ellipsoids,
    cs2_predgparamtype3f_a_pair_of_y_touching_ellipsoids,
    cs2_predgparamtype3f_a_pair_of_yz_crossed_ellipsoids,
    cs2_predgparamtype3f_a_pair_of_z_touching_ellipsoids,
    cs2_predgparamtype3f_a_y_barrel,
    cs2_predgparamtype3f_a_z_barrel,
    cs2_predgparamtype3f_a_notched_y_barrel,
    cs2_predgparamtype3f_a_notched_z_barrel,
    cs2_predgparamtype3f_a_pair_of_separate_yz_caps,

    /* toroidal */
    cs2_predgparamtype3f_a_xy_zw_torus,
    cs2_predgparamtype3f_a_xy_circle,
    cs2_predgparamtype3f_a_zw_circle,
    cs2_predgparamtype3f_a_xz_yw_torus,
    cs2_predgparamtype3f_a_xz_circle,
    cs2_predgparamtype3f_a_yw_circle,

    cs2_predgparamtype3f_COUNT
};

CS2_API const char *cs2_predgparamtype3f_str(enum cs2_predgparamtype3f_e pt);

CS2_API int cs2_predgparamtype3f_dim(enum cs2_predgparamtype3f_e pt);
CS2_API int cs2_predgparamtype3f_domain_components(enum cs2_predgparamtype3f_e pt);
CS2_API int cs2_predgparamtype3f_is_manifold(enum cs2_predgparamtype3f_e pt);
CS2_API int cs2_predgparamtype3f_has_domain_hole(enum cs2_predgparamtype3f_e pt);
CS2_API int cs2_predgparamtype3f_is_connected(enum cs2_predgparamtype3f_e pt);

struct cs2_predgparam3f_s
{
    /* parametrization type */
    enum cs2_predgparamtype3f_e t;

    /* basic properties */
    struct cs2_vec3f_s p, q, u, v;
    double a, b, c;

    /* eigenvectors */
    struct cs2_vec4f_s ev[4];

    /* eigenvalues */
    struct cs2_vec4f_s e;

};

CS2_API void cs2_predg3f_param(struct cs2_predgparam3f_s *pp, const struct cs2_predg3f_s *g);
CS2_API void cs2_predgparam3f_eval(struct cs2_spin3f_s *s, const struct cs2_predgparam3f_s *pp, double u, double v, int domain_component);

CS2_API_END

#endif /* CS2_PREDG3F_H */
