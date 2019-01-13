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
#include "cs2/predg3f.h"
#include "cs2/vec4f.h"
#include "cs2/pin3f.h"
#include "cs2/spinquad3f.h"
#include "cs2/mathf.h"
#include "cs2/assert.h"
#include <math.h>

#define EPS (10e-8)

static int _cs2_almost_zero(double x)
{
    return fabs(x) < EPS;
}

static int _cs2_almost_zero_vector(const struct cs2_vec3f_s *v)
{
    return _cs2_almost_zero(v->x) && _cs2_almost_zero(v->y) && _cs2_almost_zero(v->z);
}

static int _cs2_almost_equal(double x, double y)
{
    return _cs2_almost_zero(x - y);
}

/* clamp small perturbations near zero to a non-negative value */
static double _cs2_clamp_0(double x)
{
    if (x < 0.0)
    {
        CS2_ASSERT_MSG(x >= -EPS, "x=%.12f", x); /* guard nonsense */
        x = 0.0;
    }

    return x;
}

/* clamp small perturbations near -1 and 1 to a bounded value */
static double _cs2_clamp_11(double x)
{
    if (x < -1.0)
    {
        CS2_ASSERT_MSG(x >= -1.0 - EPS, "x=%.12f", x); /* guard nonsense */
        x = -1.0;
    }

    if (x > 1.0)
    {
        CS2_ASSERT_MSG(x <= 1.0 + EPS, "x=%.12f", x); /* guard nonsense */
        x = 1.0;
    }

    return x;
}

static void _cs2_calc_r(struct cs2_vec3f_s *r, const struct cs2_vec3f_s *v)
{
    if (!_cs2_almost_zero(v->x))
        cs2_vec3f_set(r, -v->y, v->x, v->z); /* take x-y plane */
    else if (!_cs2_almost_zero(v->y))
        cs2_vec3f_set(r, v->x, -v->z, v->y); /* take y-z plane */
    else
        cs2_vec3f_set(r, v->z, v->y, -v->x); /* take z-x plane */
}

static void _cs2_calc_ellipsoidal_w(struct cs2_vec4f_s *w, const struct cs2_vec3f_s *p, const struct cs2_vec3f_s *q, const struct cs2_vec3f_s *u, const struct cs2_vec3f_s *v, double a, double b)
{
    /*
     * w = 1 - a b p^ u^ q^ v^ - a p^ q^ - b u^ v^
     *   = 1 + a b (p^xu^) (q^xv^) - a p^ q^ - b u^ v^
     */
    struct cs2_vec3f_s ph, qh, uh, vh, phxuh, qhxvh;
    struct cs2_pin3f_s wp, ws, phqh, uhvh, phuhqhvh;

    cs2_vec3f_unit(&ph, p);
    cs2_vec3f_unit(&qh, q);
    cs2_vec3f_unit(&uh, u);
    cs2_vec3f_unit(&vh, v);
    cs2_vec3f_cross(&phxuh, &ph, &uh);
    cs2_vec3f_cross(&qhxvh, &qh, &vh);
    cs2_vec3f_cl(&phqh, &ph, &qh);
    cs2_vec3f_cl(&uhvh, &uh, &vh);
    cs2_vec3f_cl(&phuhqhvh, &phxuh, &qhxvh);
    cs2_pin3f_mad4(&wp, &CS2_PIN3F_ONE, 1.0, &phuhqhvh, a * b, &phqh, -a, &uhvh, -b);
    cs2_pin3f_mul(&ws, &wp, 0.5 / sqrt(_cs2_clamp_0(wp.p0)));
    cs2_vec4f_from_pin3f(w, &ws);
}

static void _cs2_calc_toroidal_w(struct cs2_vec4f_s *w1, struct cs2_vec4f_s *w2, const struct cs2_vec3f_s *p, const struct cs2_vec3f_s *q, double a)
{
    /*
     * z1 = [   p2 p1 - q2 q1 - a (p2 q1 - p1 q2)
     *          0
     *        - p3 p1 + q3 q1 + a (p3 q1 - p1 q3)
     *        - p1 p1 - q1 q1 + a (p1 q1 + p1 q1)
     *      ] u
     *      +
     *      [   0
     *          p2 p1 - q2 q1 - a (p2 q1 - p1 q2)
     *        - p1 p1 + q1 q1 + a (p1 q1 - p1 q1)
     *          p3 p1 + q3 q1 - a (p3 q1 + p1 q3)
     *      ] v
     *
     * z2 = [   p2 p2 - q2 q2 - a (p2 q2 - p2 q2)
     *          0
     *        - p3 p2 + q3 q2 + a (p3 q2 - p2 q3)
     *        - p1 p2 - q1 q2 + a (p1 q2 + p2 q1)
     *      ] u
     *      +
     *      [   0
     *          p2 p2 - q2 q2 - a (p2 q2 - p2 q2)
     *        - p1 p2 + q1 q2 + a (p1 q2 - p2 q1)
     *          p3 p2 + q3 q2 - a (p3 q2 + p2 q3)
     *      ] v
     *
     * z3 = [   p2 p3 - q2 q3 - a (p2 q3 - p3 q2)
     *          0
     *        - p3 p3 + q3 q3 + a (p3 q3 - p3 q3)
     *        - p1 p3 - q1 q3 + a (p1 q3 + p3 q1)
     *      ] u
     *      +
     *      [   0
     *          p2 p3 - q2 q3 - a (p2 q3 - p3 q2)
     *        - p1 p3 + q1 q3 + a (p1 q3 - p3 q1)
     *          p3 p3 + q3 q3 - a (p3 q3 + p3 q3)
     *      ] v
     *
     * assumtions:
     * [p]^2 != 0, [q]^2 !=0
     *
     * let:
     * poq = [p] / [q]
     * qop = [q] / [p]
     *
     * then:
     * [p] [q] z1 = [   qop p2 p1 - poq q2 q1 - a (p2 q1 - p1 q2)
     *                  0
     *                - qop p3 p1 + poq q3 q1 + a (p3 q1 - p1 q3)
     *                - qop p1 p1 - poq q1 q1 + a (p1 q1 + p1 q1)
     *              ] u
     *              +
     *              [   0
     *                  qop p2 p1 - poq q2 q1 - a (p2 q1 - p1 q2)
     *                - qop p1 p1 + poq q1 q1 + a (p1 q1 - p1 q1)
     *                  qop p3 p1 + poq q3 q1 - a (p3 q1 + p1 q3)
     *              ] v
     *
     * [p] [q] z2 = [   qop p2 p2 - poq q2 q2 - a (p2 q2 - p2 q2)
     *                  0
     *                - qop p3 p2 + poq q3 q2 + a (p3 q2 - p2 q3)
     *                - qop p1 p2 - poq q1 q2 + a (p1 q2 + p2 q1)
     *              ] u
     *              +
     *              [   0
     *                  qop p2 p2 - poq q2 q2 - a (p2 q2 - p2 q2)
     *                - qop p1 p2 + poq q1 q2 + a (p1 q2 - p2 q1)
     *                  qop p3 p2 + poq q3 q2 - a (p3 q2 + p2 q3)
     *              ] v
     *
     * [p] [q] z3 = [   qop p2 p3 - poq q2 q3 - a (p2 q3 - p3 q2)
     *                  0
     *                - qop p3 p3 + poq q3 q3 + a (p3 q3 - p3 q3)
     *                - qop p1 p3 - poq q1 q3 + a (p1 q3 + p3 q1)
     *              ] u
     *              +
     *              [   0
     *                  qop p2 p3 - poq q2 q3 - a (p2 q3 - p3 q2)
     *                - qop p1 p3 + poq q1 q3 + a (p1 q3 - p3 q1)
     *                  qop p3 p3 + poq q3 q3 - a (p3 q3 + p3 q3)
     *              ] v
     */
    double lp = cs2_vec3f_len(p);
    double lq = cs2_vec3f_len(q);
    double poq, qop;
    struct cs2_vec4f_s z1u, z1v, z2u, z2v, z3u, z3v;
    double mlz1, mlz2, mlz3;

    CS2_ASSERT(!_cs2_almost_zero(lp) && !_cs2_almost_zero(lq));

    poq = lp / lq;
    qop = lq / lp;

    z1u.x = qop * p->y * p->x - poq * q->y * q->x - a * (p->y * q->x - p->x * q->y);
    z1u.y = 0;
    z1u.z = - qop * p->z * p->x + poq * q->z * q->x + a * (p->z * q->x - p->x * q->z);
    z1u.w = - qop * p->x * p->x - poq * q->x * q->x + a * (p->x * q->x + p->x * q->x);

    z1v.x = 0;
    z1v.y = qop * p->y * p->x - poq * q->y * q->x - a * (p->y * q->x - p->x * q->y);
    z1v.z = - qop * p->x * p->x + poq * q->x * q->x + a * (p->x * q->x - p->x * q->x);
    z1v.w = qop * p->z * p->x + poq * q->z * q->x - a * (p->z * q->x + p->x * q->z);

    z2u.x = qop * p->y * p->y - poq * q->y * q->y - a * (p->y * q->y - p->y * q->y);
    z2u.y = 0;
    z2u.z = - qop * p->z * p->y + poq * q->z * q->y + a * (p->z * q->y - p->y * q->z);
    z2u.w = - qop * p->x * p->y - poq * q->x * q->y + a * (p->x * q->y + p->y * q->x);

    z2v.x = 0;
    z2v.y = qop * p->y * p->y - poq * q->y * q->y - a * (p->y * q->y - p->y * q->y);
    z2v.z = - qop * p->x * p->y + poq * q->x * q->y + a * (p->x * q->y - p->y * q->x);
    z2v.w = qop * p->z * p->y + poq * q->z * q->y - a * (p->z * q->y + p->y * q->z);

    z3u.x = qop * p->y * p->z - poq * q->y * q->z - a * (p->y * q->z - p->z * q->y);
    z3u.y = 0;
    z3u.z = - qop * p->z * p->z + poq * q->z * q->z + a * (p->z * q->z - p->z * q->z);
    z3u.w = - qop * p->x * p->z - poq * q->x * q->z + a * (p->x * q->z + p->z * q->x);

    z3v.x = 0;
    z3v.y = qop * p->y * p->z - poq * q->y * q->z - a * (p->y * q->z - p->z * q->y);
    z3v.z = - qop * p->x * p->z + poq * q->x * q->z + a * (p->x * q->z - p->z * q->x);
    z3v.w = qop * p->z * p->z + poq * q->z * q->z - a * (p->z * q->z + p->z * q->z);

    /* select best */
    mlz1 = CS2_MIN(cs2_vec4f_len(&z1u), cs2_vec4f_len(&z1v));
    mlz2 = CS2_MIN(cs2_vec4f_len(&z2u), cs2_vec4f_len(&z2v));
    mlz3 = CS2_MIN(cs2_vec4f_len(&z3u), cs2_vec4f_len(&z3v));

    if (mlz1 > mlz2 && mlz1 > mlz3)
    {
        cs2_vec4f_copy(w1, &z1u);
        cs2_vec4f_copy(w2, &z1v);
    }
    else if (mlz2 > mlz1 && mlz2 > mlz3)
    {
        cs2_vec4f_copy(w1, &z2u);
        cs2_vec4f_copy(w2, &z2v);
    }
    else
    {
        cs2_vec4f_copy(w1, &z3u);
        cs2_vec4f_copy(w2, &z3v);
    }

    /* normalize */
    cs2_vec4f_mul(w1, w1, 1.0 / cs2_vec4f_len(w1));
    cs2_vec4f_mul(w2, w2, 1.0 / cs2_vec4f_len(w2));
}

static enum cs2_predgparamtype3f_e _cs2_improper_param_type(void)
{
    return cs2_predgparamtype3f_an_empty_set;
}

static enum cs2_predgparamtype3f_e _cs2_ellipsoidal_param_type(double a, double b, double c)
{
    /*
     * ellipsoidal parametrization type
     *
     *  -----------------------------------------------------------------------
     *  | type     | a < b               | a = b = t/2  | a > b               |
     *  -----------------------------------------------------------------------
     *  | 1/1/1    | c ∈ (−∞, −a − b)    | c ∈ (−∞, −t) | c ∈ (−∞, −a − b)    |
     *  | 2/2/2    | c = −a − b          | c = −t       | c = −a − b          |
     *  | 3/3/3    | c ∈ (−a − b, a − b) | c ∈ (−t, 0)  | c ∈ (−a − b, b − a) |
     *  | 4/5/6    | c = a − b           | c = 0        | c = b − a           |
     *  | 7/1/8    | c ∈ (a − b, b − a)  | c ∈ ∅        | c ∈ (b − a, a − b)  |
     *  | 9/1/10   | c = b − a           | c ∈ ∅        | c = a − b           |
     *  | 11/11/11 | c ∈ (b − a, a + b)  | c ∈ (0, t)   | c ∈ (a − b, a + b)  |
     *  | 11/11/11 | c = a + b           | c = t        | c = a + b           |
     *  | 1/1/1    | c ∈ (a + b, +∞)     | c ∈ (t, +∞)  | c ∈ (a + b, +∞)     |
     *  -----------------------------------------------------------------------
     *
     *  (1) an empty set
     *  (2) a pair of points
     *  (3) a pair of separate ellipsoids
     *  (4) a pair of y-touching ellipsoids
     *  (5) a pair of yz-crossed ellipsoids
     *  (6) a pair of z-touching ellipsoids
     *  (7) a y-barrel
     *  (8) a z-barrel
     *  (9) a notched y-barrel
     *  (10) a notched z-barrel
     *  (11) a pair of separate yz-caps
     *
     *  a general rule: always approximate towards the exact cases
     */

    /* a = b = t/2 */
    if (_cs2_almost_equal(a, b))
    {
        double t = a + b;

        if (_cs2_almost_equal(c, -t))
            return cs2_predgparamtype3f_a_pair_of_points;

        if (_cs2_almost_equal(c, 0))
            return cs2_predgparamtype3f_a_pair_of_yz_crossed_ellipsoids;

        if (_cs2_almost_equal(c, t))
            return cs2_predgparamtype3f_a_pair_of_separate_yz_caps;

        if (c < -t)
            return cs2_predgparamtype3f_an_empty_set;

        if (c > -t && c < 0)
            return cs2_predgparamtype3f_a_pair_of_separate_ellipsoids;

        if (c > 0 && c < t)
            return cs2_predgparamtype3f_a_pair_of_separate_yz_caps;

        if (c > t)
            return cs2_predgparamtype3f_an_empty_set;

        /* fall through */
    }
    /* a < b */
    else if (a < b)
    {
        if (_cs2_almost_equal(c, - a - b))
            return cs2_predgparamtype3f_a_pair_of_points;

        if (_cs2_almost_equal(c, a - b))
            return cs2_predgparamtype3f_a_pair_of_y_touching_ellipsoids;

        if (_cs2_almost_equal(c, b - a))
            return cs2_predgparamtype3f_a_notched_y_barrel;

        if (_cs2_almost_equal(c, a + b))
            return cs2_predgparamtype3f_a_pair_of_separate_yz_caps;

        if (c < - a - b)
            return cs2_predgparamtype3f_an_empty_set;

        if (c > - a - b && c < a - b)
            return cs2_predgparamtype3f_a_pair_of_separate_ellipsoids;

        if (c > a - b && c < b - a)
            return cs2_predgparamtype3f_a_y_barrel;

        if (c > b - a && c < a + b)
            return cs2_predgparamtype3f_a_pair_of_separate_yz_caps;

        if (c > a + b)
            return cs2_predgparamtype3f_an_empty_set;

        /* fall through */
    }
    /* a > b */
    else if (a > b)
    {
        if (_cs2_almost_equal(c, - a - b))
            return cs2_predgparamtype3f_a_pair_of_points;

        if (_cs2_almost_equal(c, b - a))
            return cs2_predgparamtype3f_a_pair_of_z_touching_ellipsoids;

        if (_cs2_almost_equal(c, a - b))
            return cs2_predgparamtype3f_a_notched_z_barrel;

        if (_cs2_almost_equal(c, a + b))
            return cs2_predgparamtype3f_a_pair_of_separate_yz_caps;

        if (c < - a - b)
            return cs2_predgparamtype3f_an_empty_set;

        if (c > - a - b && c < b - a)
            return cs2_predgparamtype3f_a_pair_of_separate_ellipsoids;

        if (c > b - a && c < a - b)
            return cs2_predgparamtype3f_a_z_barrel;

        if (c > a - b && c < a + b)
            return cs2_predgparamtype3f_a_pair_of_separate_yz_caps;

        if (c > a + b)
            return cs2_predgparamtype3f_an_empty_set;

        /* fall through */
    }

    CS2_PANIC_MSG("unknown param type");
    return cs2_predgparamtype3f_an_empty_set;
}

static enum cs2_predgparamtype3f_e _cs2_toroidal_param_type(double a, double b, double c)
{
    /*
     * toroidal parametrization type
     *
     *  ----------------------------------------
     *  | type   | a != 0       | b != 0       |
     *  ----------------------------------------
     *  | 1/1    | c ∈ (−∞, −a) | c ∈ (−∞, −b) |
     *  | 4/7    | c = −a       | c = −b       |
     *  | 2/5    | c ∈ (−a, a)  | c ∈ (−b, b)  |
     *  | 3/6    | c = a        | c = b        |
     *  | 1/1    | c ∈ (a, ∞)   | c ∈ (b, ∞)   |
     *  ----------------------------------------
     *
     *  (1) an empty set
     *  (2) a xy/zw-torus
     *  (3) a xy-circle
     *  (4) a zw-circle
     *  (5) a xz/yw-torus
     *  (6) a xz-circle
     *  (7) a yw-circle
     *
     *  a general rule: always approximate towards the exact cases
     */

    /* a = 0 */
    if (_cs2_almost_zero(a))
    {
        if (_cs2_almost_equal(c, -b))
            return cs2_predgparamtype3f_a_yw_circle;

        if (_cs2_almost_equal(c, b))
            return cs2_predgparamtype3f_a_xz_circle;

        if (c > -b && c < b)
            return cs2_predgparamtype3f_a_xz_yw_torus;

        if (c < -b)
            return cs2_predgparamtype3f_an_empty_set;

        if (c > b)
            return cs2_predgparamtype3f_an_empty_set;

        /* fall through */
    }
    /* b = 0 */
    else if (_cs2_almost_zero(b))
    {
        if (_cs2_almost_equal(c, -a))
            return cs2_predgparamtype3f_a_zw_circle;

        if (_cs2_almost_equal(c, a))
            return cs2_predgparamtype3f_a_xy_circle;

        if (c > -a && c < a)
            return cs2_predgparamtype3f_a_xy_zw_torus;

        if (c < -a)
            return cs2_predgparamtype3f_an_empty_set;

        if (c > a)
            return cs2_predgparamtype3f_an_empty_set;

        /* fall through */
    }

    CS2_PANIC_MSG("unknown param type");
    return cs2_predgparamtype3f_COUNT;
}

static void _cs2_debug_verify_eigen_decomposition(struct cs2_mat44f_s *m, const struct cs2_predg3f_s *g)
{
    struct cs2_spinquad3f_s sp;
    int i;
    double x, y, z, w, l;
    double ex, ey, ez, ew;
    double dot, err;

    const double EPS_LEN = 10e-8;
    const double EPS_DOT = 10e-6;

    cs2_spinquad3f_from_predg3f(&sp, g);

    for (i = 0; i < 4; ++i)
    {
        ex = m->m[0][i];
        ey = m->m[1][i];
        ez = m->m[2][i];
        ew = m->m[3][i];

        x = sp.a11 * ex + sp.a12 * ey + sp.a13 * ez + sp.a14 * ew;
        y = sp.a12 * ex + sp.a22 * ey + sp.a23 * ez + sp.a24 * ew;
        z = sp.a13 * ex + sp.a23 * ey + sp.a33 * ez + sp.a34 * ew;
        w = sp.a14 * ex + sp.a24 * ey + sp.a34 * ez + sp.a44 * ew;
        l = sqrt(x * x + y * y + z * z + w * w);

        if (l < EPS_LEN)
        {
            CS2_WARN_MSG("eigenvector too small: len=%.12f, v=[%.12f, %.12f, %.12f, %.12f]^T", l, x, y, z, w);
            continue;
        }

        /* normalize */
        x /= l;
        y /= l;
        z /= l;
        w /= l;

        /* both vectors should be very close in terms of an angle */
        dot = x * ex + y * ey + z * ez + w * ew;
        err = fabs(1.0 - fabs(_cs2_clamp_11(dot)));

        CS2_ASSERT_MSG(err < EPS_DOT,
                       "failed to obtain required eigen decomposition accuracy: "
                       "target=%.12f, actual=%.12f, dot=%.12f, v=[%.12f, %.12f, %.12f, %.12f]^T, ev=[%.12f, %.12f, %.12f, %.12f]^T",
                       EPS, err, dot, x, y, z, w, ex, ey, ez, ew);
    }
}

static void _cs2_predgparam3f_eval_an_empty_set(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{
    (void)t12;
    (void)t23;
    (void)t31;
    (void)t0;

    (void)pp;

    (void)u;
    (void)v;

    (void)component;

    /* no parametrization */
    CS2_PANIC_MSG("no parametrization");
}

static void _cs2_predgparam3f_eval_a_pair_of_points(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{
    (void)pp;

    (void)u;
    (void)v;

    switch (component)
    {
        case 0:
            *t12 = 0.0;
            *t23 = 0.0;
            *t31 = 0.0;
            *t0 = 1.0;
            break;

        case 1:
            *t12 = 0.0;
            *t23 = 0.0;
            *t31 = 0.0;
            *t0 = -1.0;
            break;

        default:
            CS2_PANIC_MSG("invalid component");
            break;
    }
}

static void _cs2_predgparam3f_eval_a_pair_of_separate_ellipsoids(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{
    double sgn = 0.0;
    double r = 0.5 * (pp->a + pp->b + pp->c);
    double a, b;
    double sa, ca, sb, cb;

    switch (component)
    {
        case 0:
            sgn = 1.0;
            break;

        case 1:
            sgn = -1.0;
            v = 1 - v;
            break;

        default:
            CS2_PANIC_MSG("invalid component");
            break;
    }

    a = u * 2.0 * CS2_PI;
    b = v * CS2_PI;

    cs2_sincosf(a, &sa, &ca);
    cs2_sincosf(b, &sb, &cb);

    *t12 = sqrt(r / (pp->a + pp->b)) * sb * ca;
    *t23 = sqrt(r / pp->a) * sb * sa;
    *t31 = sqrt(r / pp->b) * cb;
    *t0 = sgn * sqrt(_cs2_clamp_0(1.0 - *t12 * *t12 - *t23 * *t23 - *t31 * *t31));
}

static void _cs2_predgparam3f_eval_a_pair_of_y_touching_ellipsoids(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{
    (void)t12;
    (void)t23;
    (void)t31;
    (void)t0;

    (void)pp;

    (void)u;
    (void)v;

    (void)component;

    /* no parametrization */
    CS2_PANIC_MSG("not implemented");
}

static void _cs2_predgparam3f_eval_a_pair_of_yz_crossed_ellipsoids(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{
    (void)t12;
    (void)t23;
    (void)t31;
    (void)t0;

    (void)pp;

    (void)u;
    (void)v;

    (void)component;

    /* no parametrization */
    CS2_PANIC_MSG("not implemented");
}

static void _cs2_predgparam3f_eval_a_pair_of_z_touching_ellipsoids(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{
    (void)t12;
    (void)t23;
    (void)t31;
    (void)t0;

    (void)pp;

    (void)u;
    (void)v;

    (void)component;

    /* no parametrization */
    CS2_PANIC_MSG("not implemented");
}

static void _cs2_predgparam3f_eval_a_y_barrel(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{
    double a, h;
    double sa, ca;
    double sgn;
    double x, y, z, d;

    CS2_ASSERT(component == 0);

    if (v >= 0.5)
    {
        sgn = 1.0;
        v = (v - 0.5) * 2.0;
    }
    else
    {
        sgn = -1.0;
        v = (0.5 - v) * 2.0;
    }

    a = u * 2 * CS2_PI;
    h = 2.0 * v - 1.0;
    cs2_sincosf(a, &sa, &ca);

    x = sqrt((pp->b - pp->a + pp->c) / (2.0 * pp->b)) * ca;
    z = sqrt((pp->b - pp->a + pp->c) / (2.0 * (pp->b - pp->a))) * sa;
    y = h * sqrt(_cs2_clamp_0(1.0 - x * x - z * z));
    d = sqrt(_cs2_clamp_0((pp->a + pp->b + pp->c) / (2.0 * ((pp->a + pp->b) * x * x + pp->a * y * y + pp->b * z * z))));

    *t12 = x * d;
    *t23 = y * d;
    *t31 = z * d;
    *t0 = sgn * sqrt(_cs2_clamp_0(1.0 - *t12 * *t12 - *t23 * *t23 - *t31 * *t31));
}

static void _cs2_predgparam3f_eval_a_z_barrel(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{
    double a, h;
    double sa, ca;
    double sgn;
    double x, y, z, d;

    CS2_ASSERT(component == 0);

    if (v >= 0.5)
    {
        sgn = 1.0;
        v = (v - 0.5) * 2.0;
    }
    else
    {
        sgn = -1.0;
        v = (0.5 - v) * 2.0;
    }

    a = u * 2 * CS2_PI;
    h = 2.0 * v - 1.0;
    cs2_sincosf(a, &sa, &ca);

    x = sqrt((pp->a - pp->b + pp->c) / (2.0 * pp->a)) * ca;
    y = sqrt((pp->a - pp->b + pp->c) / (2.0 * (pp->a - pp->b))) * sa;
    z = h * sqrt(_cs2_clamp_0(1.0 - x * x - y * y));
    d = sqrt(_cs2_clamp_0((pp->a + pp->b + pp->c) / (2.0 * ((pp->a + pp->b) * x * x + pp->a * y * y + pp->b * z * z))));

    *t12 = x * d;
    *t23 = y * d;
    *t31 = z * d;
    *t0 = sgn * sqrt(_cs2_clamp_0(1.0 - *t12 * *t12 - *t23 * *t23 - *t31 * *t31));
}

static void _cs2_predgparam3f_eval_a_notched_y_barrel(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{
    (void)t12;
    (void)t23;
    (void)t31;
    (void)t0;

    (void)pp;

    (void)u;
    (void)v;

    (void)component;

    /* no parametrization */
    CS2_PANIC_MSG("not implemented");
}

static void _cs2_predgparam3f_eval_a_notched_z_barrel(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{
    (void)t12;
    (void)t23;
    (void)t31;
    (void)t0;

    (void)pp;

    (void)u;
    (void)v;

    (void)component;

    /* no parametrization */
    CS2_PANIC_MSG("not implemented");
}

static void _cs2_predgparam3f_eval_a_pair_of_separate_yz_caps(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{
    /* yz-caps */
    double a, x, y, z, d;
    double sa, ca;
    double sgn, side = 0.0;

    switch (component)
    {
        case 0:
            side = 1.0;
            break;

        case 1:
            side = -1.0;
            v = 1 - v;
            break;

        default:
            CS2_PANIC_MSG("invalid component");
            break;
    }

    if (v >= 0.5)
    {
        sgn = 1.0;
        v = (v - 0.5) * 2.0;
    }
    else
    {
        sgn = -1.0;
        v = (0.5 - v) * 2.0;
    }

    a = u * 2 * CS2_PI;
    cs2_sincosf(a, &sa, &ca);

    y = sqrt((pp->a + pp->b - pp->c) / (2.0 * pp->b)) * ca;
    z = sqrt((pp->a + pp->b - pp->c) / (2.0 * pp->a)) * sa;
    x = side * sqrt(_cs2_clamp_0(1.0 - y * y - z * z));

    x = x * (1 - v) + side * v;
    y = y * (1 - v);
    z = z * (1 - v);

    d = sqrt(_cs2_clamp_0((pp->a + pp->b + pp->c) / (2.0 * ((pp->a + pp->b) * x * x + pp->a * y * y + pp->b * z * z))));

    *t12 = x * d;
    *t23 = y * d;
    *t31 = z * d;
    *t0 = sgn * sqrt(_cs2_clamp_0(1.0 - *t12 * *t12 - *t23 * *t23 - *t31 * *t31));
}

static void _cs2_predgparam3f_eval_a_xy_zw_torus(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{
    double alpha = u * 2 * CS2_PI;
    double beta = v * 2 * CS2_PI;
    double rp, rm, sa, ca, sb, cb;

    CS2_ASSERT(component == 0);

    rp = sqrt((pp->a + pp->c) / (2 * pp->a));
    rm = sqrt((pp->a - pp->c) / (2 * pp->a));

    cs2_sincosf(alpha, &sa, &ca);
    cs2_sincosf(beta, &sb, &cb);

    *t12 = rp * ca;
    *t23 = rp * sa;
    *t31 = rm * cb;
    *t0 = rm * sb;
}

static void _cs2_predgparam3f_eval_a_xy_circle(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{

}

static void _cs2_predgparam3f_eval_a_zw_circle(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{

}

static void _cs2_predgparam3f_eval_a_xz_yw_torus(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{
    double alpha = u * 2 * CS2_PI;
    double beta = v * 2 * CS2_PI;
    double rp, rm, sa, ca, sb, cb;

    CS2_ASSERT(component == 0);

    rp = sqrt((pp->b + pp->c) / (2 * pp->b));
    rm = sqrt((pp->b - pp->c) / (2 * pp->b));

    cs2_sincosf(alpha, &sa, &ca);
    cs2_sincosf(beta, &sb, &cb);

    *t12 = rp * ca;
    *t23 = rm * sa;
    *t31 = rp * cb;
    *t0 = rm * sb;
}

static void _cs2_predgparam3f_eval_a_xz_circle(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{

}

static void _cs2_predgparam3f_eval_a_yw_circle(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{

}


void cs2_predg3f_set(struct cs2_predg3f_s *g, const struct cs2_vec3f_s *k, const struct cs2_vec3f_s *l, const struct cs2_vec3f_s *a, const struct cs2_vec3f_s *b, double c)
{
    cs2_vec3f_copy(&g->k, k);
    cs2_vec3f_copy(&g->l, l);
    cs2_vec3f_copy(&g->a, a);
    cs2_vec3f_copy(&g->b, b);
    g->c = c;
}

void cs2_predg3f_copy(struct cs2_predg3f_s *r, const struct cs2_predg3f_s *g)
{
    cs2_vec3f_copy(&r->k, &g->k);
    cs2_vec3f_copy(&r->l, &g->l);
    cs2_vec3f_copy(&r->a, &g->a);
    cs2_vec3f_copy(&r->b, &g->b);
    r->c = g->c;
}

void cs2_predg3f_from_predh3f(struct cs2_predg3f_s *g, const struct cs2_predh3f_s *h)
{
    struct cs2_vec3f_s r, nr;

    _cs2_calc_r(&r, &h->p.n);
    cs2_vec3f_cross(&nr, &h->p.n, &r);

    g->k = nr;
    cs2_vec3f_cross(&g->l, &h->p.n, &nr);
    g->a = h->b;
    cs2_vec3f_neg(&g->b, &h->b);
    g->c = 2 * h->p.d * cs2_vec3f_sqlen(&nr);
}

void cs2_predg3f_from_preds3f(struct cs2_predg3f_s *g, const struct cs2_preds3f_s *s)
{
    g->k = s->k;
    g->l = s->l;
    g->a = s->a;
    g->b = s->b;
    g->c = 0;
}

void cs2_predg3f_pquv(struct cs2_vec3f_s *p, struct cs2_vec3f_s *q, struct cs2_vec3f_s *u, struct cs2_vec3f_s *v, const struct cs2_predg3f_s *g)
{
    cs2_vec3f_cross(p, &g->k, &g->l);
    cs2_vec3f_sub(q, &g->a, &g->b);
    cs2_vec3f_sub(u, &g->k, &g->l);
    cs2_vec3f_cross(v, &g->a, &g->b);
}

const char *cs2_predgtype3f_str(enum cs2_predgtype3f_e t)
{
    switch (t)
    {
    case cs2_predgtype3f_improper: return "improper";
    case cs2_predgtype3f_ellipsoidal: return "ellipsoidal";
    case cs2_predgtype3f_toroidal: return "toroidal";

    /* COUNT */
    case cs2_predgtype3f_COUNT: return 0;
    }

    return 0;
}

enum cs2_predgtype3f_e cs2_predg3f_type(const struct cs2_predg3f_s *g)
{
    struct cs2_vec3f_s p, q, u, v;
    double a, b;
    int za, zb;

    cs2_predg3f_pquv(&p, &q, &u, &v, g);

    a = cs2_vec3f_len(&p) * cs2_vec3f_len(&q);
    b = cs2_vec3f_len(&u) * cs2_vec3f_len(&v);
    za = _cs2_almost_zero(a);
    zb = _cs2_almost_zero(b);

    if (!za && !zb)
        return cs2_predgtype3f_ellipsoidal;
    else if (!za || !zb)
        return cs2_predgtype3f_toroidal;
    else
        return cs2_predgtype3f_improper;
}

const char *cs2_predgparamtype3f_str(enum cs2_predgparamtype3f_e pt)
{
    switch (pt)
    {
    /* common */
    case cs2_predgparamtype3f_an_empty_set: return "an empty set";

    /* ellipsoidal */
    case cs2_predgparamtype3f_a_pair_of_points: return "a pair of points";
    case cs2_predgparamtype3f_a_pair_of_separate_ellipsoids: return "a pair of separate ellipsoids";
    case cs2_predgparamtype3f_a_pair_of_y_touching_ellipsoids: return "a pair of y-touching ellipsoids";
    case cs2_predgparamtype3f_a_pair_of_yz_crossed_ellipsoids: return "a pair of yz-crossed ellipsoids";
    case cs2_predgparamtype3f_a_pair_of_z_touching_ellipsoids: return "a pair of z-touching ellipsoids";
    case cs2_predgparamtype3f_a_y_barrel: return "a y-barrel";
    case cs2_predgparamtype3f_a_z_barrel: return "a z-barrel";
    case cs2_predgparamtype3f_a_notched_y_barrel: return "a notched y-barrel";
    case cs2_predgparamtype3f_a_notched_z_barrel: return "a notched z-barrel";
    case cs2_predgparamtype3f_a_pair_of_separate_yz_caps: return "a pair of separate yz-caps";

    /* toroidal */
    case cs2_predgparamtype3f_a_xy_zw_torus: return "a xy/zw-torus";
    case cs2_predgparamtype3f_a_xy_circle: return "a xy-circle";
    case cs2_predgparamtype3f_a_zw_circle: return "a zw-circle";
    case cs2_predgparamtype3f_a_xz_yw_torus: return "a xz/yw-torus";
    case cs2_predgparamtype3f_a_xz_circle: return "a xz-circle";
    case cs2_predgparamtype3f_a_yw_circle: return "a yw-circle";

    /* COUNT */
    case cs2_predgparamtype3f_COUNT: return 0;
    }

    return 0;
}

int cs2_predgparamtype3f_dim(enum cs2_predgparamtype3f_e pt)
{
    switch (pt)
    {
    /* common */
    case cs2_predgparamtype3f_an_empty_set: return -1;

    /* ellipsoidal */
    case cs2_predgparamtype3f_a_pair_of_points: return 0;
    case cs2_predgparamtype3f_a_pair_of_separate_ellipsoids: return 2;
    case cs2_predgparamtype3f_a_pair_of_y_touching_ellipsoids: return 2; /* note: not a 2-manifold */
    case cs2_predgparamtype3f_a_pair_of_yz_crossed_ellipsoids: return 2; /* note: not a 2-manifold */
    case cs2_predgparamtype3f_a_pair_of_z_touching_ellipsoids: return 2; /* note: not a 2-manifold */
    case cs2_predgparamtype3f_a_y_barrel: return 2;
    case cs2_predgparamtype3f_a_z_barrel: return 2;
    case cs2_predgparamtype3f_a_notched_y_barrel: return 2;
    case cs2_predgparamtype3f_a_notched_z_barrel: return 2;
    case cs2_predgparamtype3f_a_pair_of_separate_yz_caps: return 2;

    /* toroidal */
    case cs2_predgparamtype3f_a_xy_zw_torus: return 2;
    case cs2_predgparamtype3f_a_xy_circle: return 1;
    case cs2_predgparamtype3f_a_zw_circle: return 1;
    case cs2_predgparamtype3f_a_xz_yw_torus: return 2;
    case cs2_predgparamtype3f_a_xz_circle: return 1;
    case cs2_predgparamtype3f_a_yw_circle: return 1;

    /* COUNT */
    case cs2_predgparamtype3f_COUNT: return -1;
    }

    CS2_PANIC_MSG("invalid param type");
    return -1;
}

int cs2_predgparamtype3f_components(enum cs2_predgparamtype3f_e pt)
{
    switch (pt)
    {
    /* common */
    case cs2_predgparamtype3f_an_empty_set: return 0;

    /* ellipsoidal */
    case cs2_predgparamtype3f_a_pair_of_points: return 2;
    case cs2_predgparamtype3f_a_pair_of_separate_ellipsoids: return 2;
    case cs2_predgparamtype3f_a_pair_of_y_touching_ellipsoids: return 1;
    case cs2_predgparamtype3f_a_pair_of_yz_crossed_ellipsoids: return 1;
    case cs2_predgparamtype3f_a_pair_of_z_touching_ellipsoids: return 1;
    case cs2_predgparamtype3f_a_y_barrel: return 1;
    case cs2_predgparamtype3f_a_z_barrel: return 1;
    case cs2_predgparamtype3f_a_notched_y_barrel: return 1;
    case cs2_predgparamtype3f_a_notched_z_barrel: return 1;
    case cs2_predgparamtype3f_a_pair_of_separate_yz_caps: return 2;

    /* toroidal */
    case cs2_predgparamtype3f_a_xy_zw_torus: return 1;
    case cs2_predgparamtype3f_a_xy_circle: return 1;
    case cs2_predgparamtype3f_a_zw_circle: return 1;
    case cs2_predgparamtype3f_a_xz_yw_torus: return 1;
    case cs2_predgparamtype3f_a_xz_circle: return 1;
    case cs2_predgparamtype3f_a_yw_circle: return 1;

    /* COUNT */
    case cs2_predgparamtype3f_COUNT: return -1;
    }

    CS2_PANIC_MSG("invalid param type");
    return -1;
}

void cs2_predg3f_param(struct cs2_predgparam3f_s *pp, const struct cs2_predg3f_s *g)
{
    struct cs2_vec3f_s p, q, u, v;
    int za, zb;

    cs2_predg3f_pquv(&p, &q, &u, &v, g);

    pp->a = cs2_vec3f_len(&p) * cs2_vec3f_len(&q);
    pp->b = cs2_vec3f_len(&u) * cs2_vec3f_len(&v);
    pp->c = g->c;

    za = _cs2_almost_zero(pp->a);
    zb = _cs2_almost_zero(pp->b);

    if (!za || !zb)
        cs2_predg3f_eigen(&pp->q, 0, g);
    else
        cs2_mat44f_zero(&pp->q);

    if (!za && !zb)
        pp->t = _cs2_ellipsoidal_param_type(pp->a, pp->b, pp->c);
    else if (!za || !zb)
        pp->t = _cs2_toroidal_param_type(pp->a, pp->b, pp->c);
    else
        pp->t = _cs2_improper_param_type();
}

void cs2_predgparam3f_eval(struct cs2_spin3f_s *s, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{
    double t12 = 0.0, t23 = 0.0, t31 = 0.0, t0 = 0.0;

    CS2_ASSERT(u >= 0.0 && u <= 1.0 && v >= 0.0 && v <= 1.0);
    CS2_ASSERT(pp->t >= 0 && pp->t < cs2_predgparamtype3f_COUNT);

    static void (* const eval_tab[cs2_predgparamtype3f_COUNT])(double *, double *, double *, double *, const struct cs2_predgparam3f_s *, double, double, int) = {
        /* common */
        _cs2_predgparam3f_eval_an_empty_set,

        /* ellipsoidal */
        _cs2_predgparam3f_eval_a_pair_of_points,
        _cs2_predgparam3f_eval_a_pair_of_separate_ellipsoids,
        _cs2_predgparam3f_eval_a_pair_of_y_touching_ellipsoids,
        _cs2_predgparam3f_eval_a_pair_of_yz_crossed_ellipsoids,
        _cs2_predgparam3f_eval_a_pair_of_z_touching_ellipsoids,
        _cs2_predgparam3f_eval_a_y_barrel,
        _cs2_predgparam3f_eval_a_z_barrel,
        _cs2_predgparam3f_eval_a_notched_y_barrel,
        _cs2_predgparam3f_eval_a_notched_z_barrel,
        _cs2_predgparam3f_eval_a_pair_of_separate_yz_caps,

        /* toroidal */
        _cs2_predgparam3f_eval_a_xy_zw_torus,
        _cs2_predgparam3f_eval_a_xy_circle,
        _cs2_predgparam3f_eval_a_zw_circle,
        _cs2_predgparam3f_eval_a_xz_yw_torus,
        _cs2_predgparam3f_eval_a_xz_circle,
        _cs2_predgparam3f_eval_a_yw_circle
    };

    eval_tab[pp->t](&t12, &t23, &t31, &t0, pp, u, v, component);

    /* eigenmatrix rotation */
    s->s12 = pp->q.m[0][0] * t12 + pp->q.m[0][1] * t23 + pp->q.m[0][2] * t31 + pp->q.m[0][3] * t0;
    s->s23 = pp->q.m[1][0] * t12 + pp->q.m[1][1] * t23 + pp->q.m[1][2] * t31 + pp->q.m[1][3] * t0;
    s->s31 = pp->q.m[2][0] * t12 + pp->q.m[2][1] * t23 + pp->q.m[2][2] * t31 + pp->q.m[2][3] * t0;
    s->s0 = pp->q.m[3][0] * t12 + pp->q.m[3][1] * t23 + pp->q.m[3][2] * t31 + pp->q.m[3][3] * t0;
}

void cs2_predg3f_eigen(struct cs2_mat44f_s *m, struct cs2_vec4f_s *e, const struct cs2_predg3f_s *g)
{
    struct cs2_vec3f_s p, q, u, v;
    struct cs2_vec4f_s w1, w2, w3, w4;
    double pl, ql, ul, vl;

    /* base */
    cs2_predg3f_pquv(&p, &q, &u, &v, g);

    /* eigenvalues */
    if (e)
    {
        pl = cs2_vec3f_len(&p);
        ql = cs2_vec3f_len(&q);
        ul = cs2_vec3f_len(&u);
        vl = cs2_vec3f_len(&v);

        e->x = g->c - (pl * ql + ul * vl);
        e->y = g->c - (pl * ql - ul * vl);
        e->z = g->c - (- pl * ql + ul * vl);
        e->w = g->c - (- pl * ql - ul * vl);
    }

    /* eigenvectors */
    if (m)
    {
        /* eigenvectors */
        switch (cs2_predg3f_type(g))
        {
            case cs2_predgtype3f_improper:
                cs2_vec4f_set(&w1, 1.0, 0.0, 0.0, 0.0);
                cs2_vec4f_set(&w2, 0.0, 1.0, 0.0, 0.0);
                cs2_vec4f_set(&w3, 0.0, 0.0, 1.0, 0.0);
                cs2_vec4f_set(&w4, 0.0, 0.0, 0.0, 1.0);

                break;

            case cs2_predgtype3f_ellipsoidal:
                _cs2_calc_ellipsoidal_w(&w1, &p, &q, &u, &v, 1.0, 1.0);
                _cs2_calc_ellipsoidal_w(&w2, &p, &q, &u, &v, 1.0, -1.0);
                _cs2_calc_ellipsoidal_w(&w3, &p, &q, &u, &v, -1.0, 1.0);
                _cs2_calc_ellipsoidal_w(&w4, &p, &q, &u, &v, -1.0, -1.0);

                break;

            case cs2_predgtype3f_toroidal:
                if (_cs2_almost_zero_vector(&p) || _cs2_almost_zero_vector(&q))
                {
                    _cs2_calc_toroidal_w(&w1, &w3, &u, &v, 1.0);
                    _cs2_calc_toroidal_w(&w2, &w4, &u, &v, -1.0);
                }
                else if (_cs2_almost_zero_vector(&u) || _cs2_almost_zero_vector(&v))
                {
                    _cs2_calc_toroidal_w(&w1, &w2, &p, &q, 1.0);
                    _cs2_calc_toroidal_w(&w3, &w4, &p, &q, -1.0);
                }
                else
                {
                    CS2_PANIC_MSG("invalid type");
                }

                break;

            /* COUNT */
            case cs2_predgtype3f_COUNT:
                CS2_PANIC_MSG("invalid type");

                break;
        }

        /* copy */
        m->m[0][0] = w1.x;
        m->m[1][0] = w1.y;
        m->m[2][0] = w1.z;
        m->m[3][0] = w1.w;

        m->m[0][1] = w2.x;
        m->m[1][1] = w2.y;
        m->m[2][1] = w2.z;
        m->m[3][1] = w2.w;

        m->m[0][2] = w3.x;
        m->m[1][2] = w3.y;
        m->m[2][2] = w3.z;
        m->m[3][2] = w3.w;

        m->m[0][3] = w4.x;
        m->m[1][3] = w4.y;
        m->m[2][3] = w4.z;
        m->m[3][3] = w4.w;

        /* debug */
        _cs2_debug_verify_eigen_decomposition(m, g);
    }
}
