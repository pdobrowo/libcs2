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
#include "cs2/mat33f.h"
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

static void _cs2_calc_r(struct cs2_vec3f_s *r, const struct cs2_vec3f_s *v)
{
    if (!_cs2_almost_zero(v->x))
        cs2_vec3f_set(r, -v->y, v->x, v->z); /* take x-y plane */
    else if (!_cs2_almost_zero(v->y))
        cs2_vec3f_set(r, v->x, -v->z, v->y); /* take y-z plane */
    else
        cs2_vec3f_set(r, v->z, v->y, -v->x); /* take z-x plane */
}

static void _cs2_debug_verify_eigen_decomposition(const struct cs2_predgparam3f_s *pp, const struct cs2_predg3f_s *g)
{
    struct cs2_spinquad3f_s sq;
    struct cs2_vec4f_s ev, tv, lv, df;
    double e, len, err;
    int i;

    const double EPS_LEN = 10e-8;
    const double EPS_DIFF = 10e-8;

    cs2_spinquad3f_from_predg3f(&sq, g);

    for (i = 0; i < 4; ++i)
    {
        /* eigenvector */
        cs2_vec4f_copy(&ev, &pp->ev[i]);

        /* eigenvalue */
        e = pp->e[i];

        /* transform eigenvector by matrix */
        cs2_vec4f_set(&tv,
                      sq.a11 * ev.x + sq.a12 * ev.y + sq.a13 * ev.z + sq.a14 * ev.w,
                      sq.a12 * ev.x + sq.a22 * ev.y + sq.a23 * ev.z + sq.a24 * ev.w,
                      sq.a13 * ev.x + sq.a23 * ev.y + sq.a33 * ev.z + sq.a34 * ev.w,
                      sq.a14 * ev.x + sq.a24 * ev.y + sq.a34 * ev.z + sq.a44 * ev.w);

        /* transform eigenvector by eigenvalue */
        cs2_vec4f_mul(&lv, &ev, e);

        /* handle zero eigenvalue */
        if (_cs2_almost_zero(fabs(e)))
        {
            len = cs2_vec4f_len(&tv);

            CS2_ASSERT_MSG(len < EPS_LEN,
                           "eigenvalue is zero but transformed eigenvector is non-zero: len=%.12f, e=%.12f, tv=[%.12f, %.12f, %.12f, %.12f]^T",
                           len, e, tv.x, tv.y, tv.z, tv.w);
            continue;
        }

        /* compare results */
        cs2_vec4f_sub(&df, &tv, &lv);
        err = cs2_vec4f_len(&df);

        CS2_ASSERT_MSG(err < EPS_DIFF,
                       "failed to obtain required eigen decomposition accuracy: "
                       "eps=%.12f, err=%.12f, tv_%d=[%.12f, %.12f, %.12f, %.12f]^T, lv_%d=[%.12f, %.12f, %.12f, %.12f]^T",
                       EPS_DIFF, err, i, tv.x, tv.y, tv.z, tv.w, i, lv.x, lv.y, lv.z, lv.w);
    }
}

static void _cs2_debug_verify_rotation_matrix(const struct cs2_predgparam3f_s *pp)
{
    double dot, err;
    int i, j;

    const double EPS_DOT = 10e-7;

    for (i = 0; i < 4; ++i)
    {
        for (j = i; j < 4; ++j)
        {
            dot = cs2_vec4f_dot(&pp->ev[i], &pp->ev[j]);

            if (i == j)
                err = fabs(1.0 - dot);
            else
                err = fabs(dot);

            CS2_ASSERT_MSG(err < EPS_DOT,
                           "failed to obtain rotation matrix: "
                           "eps=%.12f, err=%.12f, dot=%.12f, ev_%d=[%.12f, %.12f, %.12f, %.12f]^T, ev_%d=[%.12f, %.12f, %.12f, %.12f]^T",
                           EPS_DOT, err, dot, i, pp->ev[i].x, pp->ev[i].y, pp->ev[i].z, pp->ev[i].w, j, pp->ev[j].x, pp->ev[j].y, pp->ev[j].z, pp->ev[j].w);
        }
    }
}

static void _cs2_debug_verify_spinor(const struct cs2_spin3f_s *s)
{
    double len, err;

    const double EPS_LEN = 10e-10;

    len = sqrt(s->s12 * s->s12 + s->s23 * s->s23 + s->s31 * s->s31 + s->s0 * s->s0);
    err = fabs(1.0 - len);

    CS2_ASSERT_MSG(err < EPS_LEN,
                   "failed to obtain a valid spinor: "
                   "eps=%.12f, err=%.12f, len=%.12f, s=%.12f e12 + %.12f e23 + %.12f e31 + %.12f",
                   EPS_LEN, err, len, s->s12, s->s23, s->s31, s->s0);
}

static void _cs2_debug_verify_eigenvector(const struct cs2_vec4f_s *v)
{
    double len;

    const double EPS_LEN = 10e-7;

    len = cs2_vec4f_len(v);

    CS2_ASSERT_MSG(len > EPS_LEN,
                   "failed to obtain a valid non-zero eigenvector: "
                   "eps=%.12f, len=%.12f, v=[%.12f, %.12f, %.12f, %.12f]^T",
                   EPS_LEN, len, v->x, v->y, v->z, v->w);
}

static void _cs2_debug_verify_ellipsoidal_eigenpinor(const struct cs2_pin3f_s *p)
{
    double len, prop;

    const double EPS_LEN = 10e-8;

    len = cs2_pin3f_len(p);

    CS2_ASSERT_MSG(len > EPS_LEN,
                   "failed to obtain a valid ellipoidal eigen pinor: "
                   "eps=%.12f, len=%.12f, p=%.12f e12 + %.12f e23 + %.12f e31 + %.12f",
                   EPS_LEN, len, p->p12, p->p23, p->p31, p->p0);

    prop = 2.0 * sqrt(_cs2_clamp_0(p->p0));

    CS2_ASSERT_MSG(fabs(len - prop) < EPS_LEN,
                   "failed to check property of scalar in ellipoidal eigen pinor: "
                   "eps=%.12f, len=%.12f, prop=%.12f, p=%.12f e12 + %.12f e23 + %.12f e31 + %.12f",
                   EPS_LEN, len, prop, p->p12, p->p23, p->p31, p->p0);
}

void _cs2_debug_verify_inverse_pquvc(const struct cs2_spinquad3f_s *sq, const struct cs2_vec3f_s *p, const struct cs2_vec3f_s *q, const struct cs2_vec3f_s *u, const struct cs2_vec3f_s *v, double c)
{
    struct cs2_vec3f_s pxq, uxv, r;
    struct cs2_spinquad3f_s tsq, nsq;
    double t, pu, qv;

    const double EPS_LEN = 10e-8;

    cs2_vec3f_cross(&pxq, p, q);
    cs2_vec3f_cross(&uxv, u, v);
    cs2_vec3f_add(&r, &pxq, &uxv);
    t = cs2_vec3f_dot(p, q) + cs2_vec3f_dot(u, v);
    tsq.a11 = 2.0 * (p->z * q->z + u->z * v->z) - t + c;
    tsq.a12 = 2.0 * (p->x * q->z + u->x * v->z) + r.y;
    tsq.a13 = 2.0 * (p->z * q->y + u->z * v->y) + r.x;
    tsq.a14 = r.z;
    tsq.a22 = 2.0 * (p->x * q->x + u->x * v->x) - t + c;
    tsq.a23 = 2.0 * (p->y * q->x + u->y * v->x) + r.z;
    tsq.a24 = r.x;
    tsq.a33 = 2.0 * (p->y * q->y + u->y * v->y) - t + c;
    tsq.a34 = r.y;
    tsq.a44 = t + c;
    pu = cs2_vec3f_dot(p, u);
    qv = cs2_vec3f_dot(q, v);

    cs2_spinquad3f_unit(&nsq, sq);
    cs2_spinquad3f_unit(&tsq, &tsq);

    CS2_ASSERT_MSG(fabs(nsq.a11 - tsq.a11) < EPS_LEN,
                   "failed to check solution to inverse spin quadric: "
                   "eps=%.12f, sq_a11=%.12f, a11=%.12f",
                   EPS_LEN, nsq.a11, tsq.a11);

    CS2_ASSERT_MSG(fabs(nsq.a12 - tsq.a12) < EPS_LEN,
                   "failed to check solution to inverse spin quadric: "
                   "eps=%.12f, sq_a12=%.12f, a12=%.12f",
                   EPS_LEN, nsq.a12, tsq.a12);

    CS2_ASSERT_MSG(fabs(nsq.a13 - tsq.a13) < EPS_LEN,
                   "failed to check solution to inverse spin quadric: "
                   "eps=%.12f, sq_a13=%.12f, a13=%.12f",
                   EPS_LEN, nsq.a13, tsq.a13);

    CS2_ASSERT_MSG(fabs(nsq.a14 - tsq.a14) < EPS_LEN,
                   "failed to check solution to inverse spin quadric: "
                   "eps=%.12f, sq_a14=%.12f, a14=%.12f",
                   EPS_LEN, nsq.a14, tsq.a14);

    CS2_ASSERT_MSG(fabs(nsq.a22 - tsq.a22) < EPS_LEN,
                   "failed to check solution to inverse spin quadric: "
                   "eps=%.12f, sq_a22=%.12f, a22=%.12f",
                   EPS_LEN, nsq.a22, tsq.a22);

    CS2_ASSERT_MSG(fabs(nsq.a23 - tsq.a23) < EPS_LEN,
                   "failed to check solution to inverse spin quadric: "
                   "eps=%.12f, sq_a23=%.12f, a23=%.12f",
                   EPS_LEN, nsq.a23, tsq.a23);

    CS2_ASSERT_MSG(fabs(nsq.a24 - tsq.a24) < EPS_LEN,
                   "failed to check solution to inverse spin quadric: "
                   "eps=%.12f, sq_a24=%.12f, a24=%.12f",
                   EPS_LEN, nsq.a24, tsq.a24);

    CS2_ASSERT_MSG(fabs(nsq.a33 - tsq.a33) < EPS_LEN,
                   "failed to check solution to inverse spin quadric: "
                   "eps=%.12f, sq_a33=%.12f, a33=%.12f",
                   EPS_LEN, nsq.a33, tsq.a33);

    CS2_ASSERT_MSG(fabs(nsq.a34 - tsq.a34) < EPS_LEN,
                   "failed to check solution to inverse spin quadric: "
                   "eps=%.12f, sq_a34=%.12f, a34=%.12f",
                   EPS_LEN, nsq.a34, tsq.a34);

    CS2_ASSERT_MSG(fabs(nsq.a44 - tsq.a44) < EPS_LEN,
                   "failed to check solution to inverse spin quadric: "
                   "eps=%.12f, sq_a44=%.12f, a44=%.12f",
                   EPS_LEN, nsq.a44, tsq.a44);

    CS2_ASSERT_MSG(fabs(pu) < EPS_LEN,
                   "failed to check solution to inverse spin quadric: "
                   "eps=%.12f, pu=%.12f",
                   EPS_LEN, pu);

    CS2_ASSERT_MSG(fabs(qv) < EPS_LEN,
                   "failed to check solution to inverse spin quadric: "
                   "eps=%.12f, qv=%.12f",
                   EPS_LEN, qv);
}

static void _cs2_calc_ellipsoidal_eigenvector(struct cs2_vec4f_s *w, const struct cs2_vec3f_s *p, const struct cs2_vec3f_s *q, const struct cs2_vec3f_s *u, const struct cs2_vec3f_s *v, double a, double b)
{
    /*
     * w = 1 - a b p^ u^ q^ v^ - a p^ q^ - b u^ v^
     *   = 1 + a b (p^xu^) (q^xv^) - a p^ q^ - b u^ v^
     */
    struct cs2_vec3f_s ph, qh, uh, vh, phxuh, qhxvh;
    struct cs2_pin3f_s wp, ws, phqh, uhvh, phxuhqhxvh;

    cs2_vec3f_unit(&ph, p);
    cs2_vec3f_unit(&qh, q);
    cs2_vec3f_unit(&uh, u);
    cs2_vec3f_unit(&vh, v);
    cs2_vec3f_cross(&phxuh, &ph, &uh);
    cs2_vec3f_cross(&qhxvh, &qh, &vh);
    cs2_pin3f_cl(&phqh, &ph, &qh);
    cs2_pin3f_cl(&uhvh, &uh, &vh);
    cs2_pin3f_cl(&phxuhqhxvh, &phxuh, &qhxvh);
    cs2_pin3f_mad4(&wp, &CS2_PIN3F_ONE, 1.0, &phxuhqhxvh, a * b, &phqh, -a, &uhvh, -b);
    _cs2_debug_verify_ellipsoidal_eigenpinor(&wp);
    cs2_pin3f_mul(&ws, &wp, 0.5 / sqrt(_cs2_clamp_0(wp.p0)));
    cs2_vec4f_from_pin3f(w, &ws);
}

static void _cs2_calc_toroidal_eigenplane(struct cs2_vec4f_s *w1, struct cs2_vec4f_s *w2, const struct cs2_vec3f_s *p, const struct cs2_vec3f_s *q, double a)
{
    /*
     * p, q: normalized
     *
     * Z = [
     *       + p2 + a q2
     *       0
     *       - p3 - a q3
     *       - p1 + a q1
     *     ] u
     *     +
     *     [
     *       0
     *       + p2 + a q2
     *       - p1 - a q1
     *       + p3 - a q3
     *     ] v
     */
    struct cs2_vec3f_s np, nq;
    struct cs2_vec4f_s zu, zv;
    struct cs2_vec4f_s ozu, ozv;

    cs2_vec3f_unit(&np, p);
    cs2_vec3f_unit(&nq, q);

    cs2_vec4f_set(&zu,
                  + np.y + a * nq.y,
                  0,
                  - np.z - a * nq.z,
                  - np.x + a * nq.x);

    cs2_vec4f_set(&zv,
                  0,
                  + np.y + a * nq.y,
                  - np.x - a * nq.x,
                  + np.z - a * nq.z);

    /* TODO: handle zu = 0 or zv = 0 */

    /* orthogonalize */
    cs2_vec4f_copy(&ozu, &zu);
    cs2_vec4f_mad2(&ozv, &zv, 1.0, &zu, -cs2_vec4f_dot(&zu, &zv) / cs2_vec4f_sqlen(&zu));

    /* normalize */
    cs2_vec4f_unit(w1, &ozu);
    cs2_vec4f_unit(w2, &ozv);
}

static void _cs2_calc_eigen_decomposition(struct cs2_predgparam3f_s *pp, const struct cs2_predg3f_s *g)
{
    int i;

    /* eigenvalues */
    pp->e[0] = pp->c - (pp->a + pp->b);
    pp->e[1] = pp->c - (pp->a - pp->b);
    pp->e[2] = pp->c - (-pp->a + pp->b);
    pp->e[3] = pp->c - (-pp->a - pp->b);

    /* eigenvectors */
    switch (cs2_predg3f_type(g))
    {
        case cs2_predgtype3f_improper:
            cs2_vec4f_set(&pp->ev[0], 1.0, 0.0, 0.0, 0.0);
            cs2_vec4f_set(&pp->ev[1], 0.0, 1.0, 0.0, 0.0);
            cs2_vec4f_set(&pp->ev[2], 0.0, 0.0, 1.0, 0.0);
            cs2_vec4f_set(&pp->ev[3], 0.0, 0.0, 0.0, 1.0);

            break;

        case cs2_predgtype3f_ellipsoidal:
            _cs2_calc_ellipsoidal_eigenvector(&pp->ev[0], &pp->p, &pp->q, &pp->u, &pp->v, 1.0, 1.0);
            _cs2_calc_ellipsoidal_eigenvector(&pp->ev[1], &pp->p, &pp->q, &pp->u, &pp->v, 1.0, -1.0);
            _cs2_calc_ellipsoidal_eigenvector(&pp->ev[2], &pp->p, &pp->q, &pp->u, &pp->v, -1.0, 1.0);
            _cs2_calc_ellipsoidal_eigenvector(&pp->ev[3], &pp->p, &pp->q, &pp->u, &pp->v, -1.0, -1.0);

            break;

        case cs2_predgtype3f_toroidal:
            if (_cs2_almost_zero_vector(&pp->p) || _cs2_almost_zero_vector(&pp->q))
            {
                _cs2_calc_toroidal_eigenplane(&pp->ev[0], &pp->ev[2], &pp->u, &pp->v, 1.0);
                _cs2_calc_toroidal_eigenplane(&pp->ev[1], &pp->ev[3], &pp->u, &pp->v, -1.0);
            }
            else if (_cs2_almost_zero_vector(&pp->u) || _cs2_almost_zero_vector(&pp->v))
            {
                _cs2_calc_toroidal_eigenplane(&pp->ev[0], &pp->ev[1], &pp->p, &pp->q, 1.0);
                _cs2_calc_toroidal_eigenplane(&pp->ev[2], &pp->ev[3], &pp->p, &pp->q, -1.0);
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

    /* debug */
    for (i = 0; i < 4; ++i)
        _cs2_debug_verify_eigenvector(&pp->ev[i]);

    _cs2_debug_verify_eigen_decomposition(pp, g);
    _cs2_debug_verify_rotation_matrix(pp);
}

static void _cs2_improper_eigen_decomposition(struct cs2_predgparam3f_s *pp, const struct cs2_predg3f_s *g)
{
    cs2_vec4f_set(&pp->ev[0], 1.0, 0.0, 0.0, 0.0);
    cs2_vec4f_set(&pp->ev[1], 0.0, 1.0, 0.0, 0.0);
    cs2_vec4f_set(&pp->ev[2], 0.0, 0.0, 1.0, 0.0);
    cs2_vec4f_set(&pp->ev[3], 0.0, 0.0, 0.0, 1.0);

    pp->e[0] = g->c;
    pp->e[1] = g->c;
    pp->e[2] = g->c;
    pp->e[3] = g->c;
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

static void _cs2_predgparam3f_eval_an_empty_set(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int domain_component)
{
    (void)t12;
    (void)t23;
    (void)t31;
    (void)t0;

    (void)pp;

    (void)u;
    (void)v;

    (void)domain_component;

    /* no parametrization */
    CS2_PANIC_MSG("no parametrization");
}

static void _cs2_predgparam3f_eval_a_pair_of_points(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int domain_component)
{
    (void)pp;

    (void)u;
    (void)v;

    switch (domain_component)
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

static void _cs2_predgparam3f_eval_a_pair_of_separate_ellipsoids(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int domain_component)
{
    double sgn = 0.0;
    double r = 0.5 * (pp->a + pp->b + pp->c);
    double a, b;
    double sa, ca, sb, cb;

    switch (domain_component)
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

    cs2_sincosf(&sa, &ca, a);
    cs2_sincosf(&sb, &cb, b);

    *t12 = sqrt(r / (pp->a + pp->b)) * sb * ca;
    *t23 = sqrt(r / pp->a) * sb * sa;
    *t31 = sqrt(r / pp->b) * cb;
    *t0 = sgn * sqrt(_cs2_clamp_0(1.0 - *t12 * *t12 - *t23 * *t23 - *t31 * *t31));
}

static void _cs2_predgparam3f_eval_a_pair_of_y_touching_ellipsoids(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int domain_component)
{
    /* same as 'a pair of separate ellipsoids'
     * includes a domain hole
     */
    _cs2_predgparam3f_eval_a_pair_of_separate_ellipsoids(t12, t23, t31, t0, pp, u, v, domain_component);
}

static void _cs2_predgparam3f_eval_a_pair_of_yz_crossed_ellipsoids(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int domain_component)
{
    /* same as 'a pair of separate ellipsoids'
     * includes a domain hole
     */
    _cs2_predgparam3f_eval_a_pair_of_separate_ellipsoids(t12, t23, t31, t0, pp, u, v, domain_component);
}

static void _cs2_predgparam3f_eval_a_pair_of_z_touching_ellipsoids(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int domain_component)
{
    /* same as 'a pair of separate ellipsoids'
     * includes a domain hole
     */
    _cs2_predgparam3f_eval_a_pair_of_separate_ellipsoids(t12, t23, t31, t0, pp, u, v, domain_component);
}

static void _cs2_predgparam3f_eval_a_y_barrel(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int domain_component)
{
    double a, h;
    double sa, ca;
    double sgn;
    double x, y, z, d;

    CS2_ASSERT_MSG(domain_component == 0, "invalid component");

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

    a = u * 2.0 * CS2_PI;
    h = 2.0 * v - 1.0;
    cs2_sincosf(&sa, &ca, a);

    x = sqrt((pp->b - pp->a + pp->c) / (2.0 * pp->b)) * ca;
    z = sqrt((pp->b - pp->a + pp->c) / (2.0 * (pp->b - pp->a))) * sa;
    y = h * sqrt(_cs2_clamp_0(1.0 - x * x - z * z));
    d = sqrt(_cs2_clamp_0((pp->a + pp->b + pp->c) / (2.0 * ((pp->a + pp->b) * x * x + pp->a * y * y + pp->b * z * z))));

    *t12 = x * d;
    *t23 = y * d;
    *t31 = z * d;
    *t0 = sgn * sqrt(_cs2_clamp_0(1.0 - *t12 * *t12 - *t23 * *t23 - *t31 * *t31));
}

static void _cs2_predgparam3f_eval_a_z_barrel(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int domain_component)
{
    double a, h;
    double sa, ca;
    double sgn;
    double x, y, z, d;

    CS2_ASSERT_MSG(domain_component == 0, "invalid component");

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

    a = u * 2.0 * CS2_PI;
    h = 2.0 * v - 1.0;
    cs2_sincosf(&sa, &ca, a);

    x = sqrt((pp->a - pp->b + pp->c) / (2.0 * pp->a)) * ca;
    y = sqrt((pp->a - pp->b + pp->c) / (2.0 * (pp->a - pp->b))) * sa;
    z = h * sqrt(_cs2_clamp_0(1.0 - x * x - y * y));
    d = sqrt(_cs2_clamp_0((pp->a + pp->b + pp->c) / (2.0 * ((pp->a + pp->b) * x * x + pp->a * y * y + pp->b * z * z))));

    *t12 = x * d;
    *t23 = y * d;
    *t31 = z * d;
    *t0 = sgn * sqrt(_cs2_clamp_0(1.0 - *t12 * *t12 - *t23 * *t23 - *t31 * *t31));
}

static void _cs2_predgparam3f_eval_a_notched_y_barrel(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int domain_component)
{
    /* same as 'a y-barrel'
     * includes a domain hole
     */
    _cs2_predgparam3f_eval_a_y_barrel(t12, t23, t31, t0, pp, u, v, domain_component);
}

static void _cs2_predgparam3f_eval_a_notched_z_barrel(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int domain_component)
{
    /* same as 'a z-barrel'
     * includes a domain hole
     */
    _cs2_predgparam3f_eval_a_z_barrel(t12, t23, t31, t0, pp, u, v, domain_component);
}

static void _cs2_predgparam3f_eval_a_pair_of_separate_yz_caps(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int domain_component)
{
    /* yz-caps */
    double a, x, y, z, d;
    double sa, ca;
    double sgn, side = 0.0;

    switch (domain_component)
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

    a = u * 2.0 * CS2_PI;
    cs2_sincosf(&sa, &ca, a);

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

static void _cs2_predgparam3f_eval_a_xy_zw_torus(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int domain_component)
{
    double alpha = u * 2 * CS2_PI;
    double beta = v * 2 * CS2_PI;
    double rp, rm, sa, ca, sb, cb;

    CS2_ASSERT_MSG(domain_component == 0, "invalid component");

    rp = sqrt((pp->a + pp->c) / (2.0 * pp->a));
    rm = sqrt((pp->a - pp->c) / (2.0 * pp->a));

    cs2_sincosf(&sa, &ca, alpha);
    cs2_sincosf(&sb, &cb, beta);

    *t12 = rp * ca;
    *t23 = rp * sa;
    *t31 = rm * cb;
    *t0 = rm * sb;
}

static void _cs2_predgparam3f_eval_a_xy_circle(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int domain_component)
{
    (void)pp;
    (void)v;

    double alpha = u * 2 * CS2_PI;
    double sa, ca;

    CS2_ASSERT_MSG(domain_component == 0, "invalid component");

    cs2_sincosf(&sa, &ca, alpha);

    *t12 = ca;
    *t23 = sa;
    *t31 = 0.0;
    *t0 = 0.0;
}

static void _cs2_predgparam3f_eval_a_zw_circle(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int domain_component)
{
    (void)pp;
    (void)v;

    double alpha = u * 2 * CS2_PI;
    double sa, ca;

    CS2_ASSERT_MSG(domain_component == 0, "invalid component");

    cs2_sincosf(&sa, &ca, alpha);

    *t12 = 0.0;
    *t23 = 0.0;
    *t31 = ca;
    *t0 = sa;
}

static void _cs2_predgparam3f_eval_a_xz_yw_torus(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int domain_component)
{
    double alpha = u * 2 * CS2_PI;
    double beta = v * 2 * CS2_PI;
    double rp, rm, sa, ca, sb, cb;

    CS2_ASSERT_MSG(domain_component == 0, "invalid component");

    rp = sqrt((pp->b + pp->c) / (2.0 * pp->b));
    rm = sqrt((pp->b - pp->c) / (2.0 * pp->b));

    cs2_sincosf(&sa, &ca, alpha);
    cs2_sincosf(&sb, &cb, beta);

    *t12 = rp * ca;
    *t23 = rm * cb;
    *t31 = rp * sa;
    *t0 = rm * sb;
}

static void _cs2_predgparam3f_eval_a_xz_circle(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int domain_component)
{
    (void)pp;
    (void)v;

    double alpha = u * 2 * CS2_PI;
    double sa, ca;

    CS2_ASSERT_MSG(domain_component == 0, "invalid component");

    cs2_sincosf(&sa, &ca, alpha);

    *t12 = ca;
    *t23 = 0.0;
    *t31 = sa;
    *t0 = 0.0;
}

static void _cs2_predgparam3f_eval_a_yw_circle(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int domain_component)
{
    (void)pp;
    (void)v;

    double alpha = u * 2 * CS2_PI;
    double sa, ca;

    CS2_ASSERT_MSG(domain_component == 0, "invalid component");

    cs2_sincosf(&sa, &ca, alpha);

    *t12 = 0.0;
    *t23 = ca;
    *t31 = 0.0;
    *t0 = sa;
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

void cs2_predg3f_from_pquvc(struct cs2_predg3f_s *pg, const struct cs2_vec3f_s *p, const struct cs2_vec3f_s *q, const struct cs2_vec3f_s *u, const struct cs2_vec3f_s *v, double c, double alpha, double beta)
{
    struct cs2_vec3f_s pxu, vxq;
    double usl = cs2_vec3f_sqlen(u);
    double qsl = cs2_vec3f_sqlen(q);

    CS2_ASSERT_MSG(!_cs2_almost_zero(usl) && !_cs2_almost_zero(qsl), "improper pred");

    cs2_vec3f_cross(&pxu, p, u);
    cs2_vec3f_cross(&vxq, v, q);

    cs2_vec3f_mad2(&pg->l, u, alpha, &pxu, 1.0 / usl);
    cs2_vec3f_add(&pg->k, &pg->l, u);
    cs2_vec3f_mad2(&pg->b, q, beta, &vxq, 1.0 / qsl);
    cs2_vec3f_add(&pg->a, &pg->b, q);
    pg->c = c;
}

void cs2_predg3f_from_spinquad3f(struct cs2_predg3f_s *pg, const struct cs2_spinquad3f_s *sq, int sign, double alpha, double beta, double mu, double nu)
{
    double p1 = sq->a11 - sq->a22 + sq->a33 - sq->a44;
    double p2 = 2.0 * (sq->a12 + sq->a34);
    double p3 = 2.0 * (sq->a13 + sq->a24);
    double p4 = 2.0 * (sq->a14 + sq->a23);
    double m1 = sq->a11 - sq->a22 - sq->a33 + sq->a44;
    double m2 = 2.0 * (sq->a12 - sq->a34);
    double m3 = 2.0 * (sq->a13 - sq->a24);
    double m4 = 2.0 * (sq->a14 - sq->a23);
    double t = sq->a11 + sq->a22 - sq->a33 - sq->a44;
    double c = 0.25 * (sq->a11 + sq->a22 + sq->a33 + sq->a44);
    struct cs2_vec3f_s g = { m2, p3, m1 };
    struct cs2_vec3f_s h = { p1, m4, -p2 };
    struct cs2_vec3f_s j = { p4, -t, m3 };
    struct cs2_vec3f_s p, q, u, v;
    struct cs2_mat33f_s m = { -h.x, -h.y, -h.z,
                              j.x, j.y, j.z,
                              g.x, g.y, g.z };
    struct cs2_vec3f_s gxh, hxj, jxg, gwph, ngxh, ngwph, ngwphxngxh;
    double gxhgxh, hxjhxj, jxgjxg, gg, hh, jj, gh, jg;
    double g1, g2, gs, w, cond;

    const double COND_EPS = 10e-5;

    cs2_vec3f_cross(&gxh, &g, &h);
    cs2_vec3f_cross(&hxj, &h, &j);
    cs2_vec3f_cross(&jxg, &j, &g);

    gg = cs2_vec3f_sqlen(&g);
    hh = cs2_vec3f_sqlen(&h);
    jj = cs2_vec3f_sqlen(&j);

    gxhgxh = cs2_vec3f_sqlen(&gxh);
    hxjhxj = cs2_vec3f_sqlen(&hxj);
    jxgjxg = cs2_vec3f_sqlen(&jxg);

    gh = cs2_vec3f_dot(&g, &h);
    jg = cs2_vec3f_dot(&j, &g);

    cond = cs2_vec3f_dot(&gxh, &j);

    CS2_ASSERT(fabs(cond) < COND_EPS);

    gs = (gg + hh + jj) * (gg + hh + jj) - 4.0 * (gxhgxh + hxjhxj + jxgjxg);
    CS2_ASSERT(gs >= 0.0);

    g1 = -gh * gxhgxh + jg * cs2_vec3f_dot(&jxg, &gxh);
    CS2_ASSERT(fabs(g1) > 0.0);

    g2 = gg * (gxhgxh + hxjhxj) - hh * (gxhgxh + jxgjxg);

    w = (-g2 + sign * gxhgxh * sqrt(gs)) / (2.0 * g1);

    cs2_vec3f_mad2(&gwph, &g, w, &h, 1.0);

    CS2_ASSERT(cs2_vec3f_sqlen(&gxh) > 0.0);
    CS2_ASSERT(cs2_vec3f_sqlen(&gwph) > 0.0);

    cs2_vec3f_unit(&ngxh, &gxh);
    cs2_vec3f_unit(&ngwph, &gwph);

    cs2_vec3f_cross(&ngwphxngxh, &ngwph, &ngxh);

    cs2_vec3f_mul(&p, &ngwph, 1.0 / (2.0 * mu));
    cs2_vec3f_mul(&u, &ngwphxngxh, 1.0 / (2.0 * nu));

    cs2_mat33f_transform(&q, &m, &ngwph);
    cs2_vec3f_mul(&q, &q, mu / 2.0);

    cs2_mat33f_transform(&v, &m, &ngwphxngxh);
    cs2_vec3f_mul(&v, &v, nu / 2.0);

    _cs2_debug_verify_inverse_pquvc(sq, &p, &q, &u, &v, c);

    cs2_predg3f_from_pquvc(pg, &p, &q, &u, &v, c, alpha, beta);
}

const char *cs2_predgtype3f_str(enum cs2_predgtype3f_e pgt)
{
    switch (pgt)
    {
    case cs2_predgtype3f_improper: return "improper";
    case cs2_predgtype3f_ellipsoidal: return "ellipsoidal";
    case cs2_predgtype3f_toroidal: return "toroidal";

    /* COUNT */
    case cs2_predgtype3f_COUNT: return 0;
    }

    return 0;
}

enum cs2_predgtype3f_e cs2_predg3f_type(const struct cs2_predg3f_s *pg)
{
    struct cs2_vec3f_s p, q, u, v;
    double a, b;
    int za, zb;

    cs2_predg3f_pquv(&p, &q, &u, &v, pg);

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

const char *cs2_predgparamtype3f_str(enum cs2_predgparamtype3f_e pgpt)
{
    switch (pgpt)
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

int cs2_predgparamtype3f_dim(enum cs2_predgparamtype3f_e pgpt)
{
    switch (pgpt)
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

int cs2_predgparamtype3f_domain_components(enum cs2_predgparamtype3f_e pgpt)
{
    switch (pgpt)
    {
    /* common */
    case cs2_predgparamtype3f_an_empty_set: return 0;

    /* ellipsoidal */
    case cs2_predgparamtype3f_a_pair_of_points: return 2;
    case cs2_predgparamtype3f_a_pair_of_separate_ellipsoids: return 2;
    case cs2_predgparamtype3f_a_pair_of_y_touching_ellipsoids: return 2;
    case cs2_predgparamtype3f_a_pair_of_yz_crossed_ellipsoids: return 2;
    case cs2_predgparamtype3f_a_pair_of_z_touching_ellipsoids: return 2;
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

int cs2_predgparamtype3f_is_manifold(enum cs2_predgparamtype3f_e pgpt)
{
    switch (pgpt)
    {
    /* common */
    case cs2_predgparamtype3f_an_empty_set: return 1;

    /* ellipsoidal */
    case cs2_predgparamtype3f_a_pair_of_points: return 1;
    case cs2_predgparamtype3f_a_pair_of_separate_ellipsoids: return 1;
    case cs2_predgparamtype3f_a_pair_of_y_touching_ellipsoids: return 0;
    case cs2_predgparamtype3f_a_pair_of_yz_crossed_ellipsoids: return 0;
    case cs2_predgparamtype3f_a_pair_of_z_touching_ellipsoids: return 0;
    case cs2_predgparamtype3f_a_y_barrel: return 1;
    case cs2_predgparamtype3f_a_z_barrel: return 1;
    case cs2_predgparamtype3f_a_notched_y_barrel: return 1;
    case cs2_predgparamtype3f_a_notched_z_barrel: return 1;
    case cs2_predgparamtype3f_a_pair_of_separate_yz_caps: return 1;

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

int cs2_predgparamtype3f_has_domain_hole(enum cs2_predgparamtype3f_e pgpt)
{
    switch (pgpt)
    {
    /* common */
    case cs2_predgparamtype3f_an_empty_set: return 0;

    /* ellipsoidal */
    case cs2_predgparamtype3f_a_pair_of_points: return 0;
    case cs2_predgparamtype3f_a_pair_of_separate_ellipsoids: return 0;
    case cs2_predgparamtype3f_a_pair_of_y_touching_ellipsoids: return 1;
    case cs2_predgparamtype3f_a_pair_of_yz_crossed_ellipsoids: return 1;
    case cs2_predgparamtype3f_a_pair_of_z_touching_ellipsoids: return 1;
    case cs2_predgparamtype3f_a_y_barrel: return 1;
    case cs2_predgparamtype3f_a_z_barrel: return 1;
    case cs2_predgparamtype3f_a_notched_y_barrel: return 1;
    case cs2_predgparamtype3f_a_notched_z_barrel: return 1;
    case cs2_predgparamtype3f_a_pair_of_separate_yz_caps: return 0;

    /* toroidal */
    case cs2_predgparamtype3f_a_xy_zw_torus: return 0;
    case cs2_predgparamtype3f_a_xy_circle: return 0;
    case cs2_predgparamtype3f_a_zw_circle: return 0;
    case cs2_predgparamtype3f_a_xz_yw_torus: return 0;
    case cs2_predgparamtype3f_a_xz_circle: return 0;
    case cs2_predgparamtype3f_a_yw_circle: return 0;

    /* COUNT */
    case cs2_predgparamtype3f_COUNT: return -1;
    }

    CS2_PANIC_MSG("invalid param type");
    return -1;
}

int cs2_predgparamtype3f_is_connected(enum cs2_predgparamtype3f_e pgpt)
{
    switch (pgpt)
    {
    /* common */
    case cs2_predgparamtype3f_an_empty_set: return 1;

    /* ellipsoidal */
    case cs2_predgparamtype3f_a_pair_of_points: return 0;
    case cs2_predgparamtype3f_a_pair_of_separate_ellipsoids: return 0;
    case cs2_predgparamtype3f_a_pair_of_y_touching_ellipsoids: return 1;
    case cs2_predgparamtype3f_a_pair_of_yz_crossed_ellipsoids: return 1;
    case cs2_predgparamtype3f_a_pair_of_z_touching_ellipsoids: return 1;
    case cs2_predgparamtype3f_a_y_barrel: return 1;
    case cs2_predgparamtype3f_a_z_barrel: return 1;
    case cs2_predgparamtype3f_a_notched_y_barrel: return 1;
    case cs2_predgparamtype3f_a_notched_z_barrel: return 1;
    case cs2_predgparamtype3f_a_pair_of_separate_yz_caps: return 0;

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

void cs2_predg3f_param(struct cs2_predgparam3f_s *pgp, const struct cs2_predg3f_s *pg)
{
    int za, zb;

    /* basic properties */
    cs2_predg3f_pquv(&pgp->p, &pgp->q, &pgp->u, &pgp->v, pg);

    pgp->a = cs2_vec3f_len(&pgp->p) * cs2_vec3f_len(&pgp->q);
    pgp->b = cs2_vec3f_len(&pgp->u) * cs2_vec3f_len(&pgp->v);
    pgp->c = pg->c;

    /* non-zero characteristics */
    za = _cs2_almost_zero(pgp->a);
    zb = _cs2_almost_zero(pgp->b);

    /* parametrization type */
    if (!za && !zb)
        pgp->t = _cs2_ellipsoidal_param_type(pgp->a, pgp->b, pgp->c);
    else if (!za || !zb)
        pgp->t = _cs2_toroidal_param_type(pgp->a, pgp->b, pgp->c);
    else
        pgp->t = _cs2_improper_param_type();

    /* eigen decomposition */
    if (!za || !zb)
        _cs2_calc_eigen_decomposition(pgp, pg);
    else
        _cs2_improper_eigen_decomposition(pgp, pg);
}

void cs2_predgparam3f_eval(struct cs2_spin3f_s *s, const struct cs2_predgparam3f_s *pgp, double u, double v, int domain_component)
{
    double t12, t23, t31, t0;

    CS2_ASSERT_MSG(u >= 0.0 && u <= 1.0 && v >= 0.0 && v <= 1.0, "param outside domain");
    CS2_ASSERT_MSG(pgp->t >= 0 && pgp->t < cs2_predgparamtype3f_COUNT, "invalid param type");

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

    eval_tab[pgp->t](&t12, &t23, &t31, &t0, pgp, u, v, domain_component);

    /* eigenmatrix rotation */
    s->s12 = pgp->ev[0].x * t12 + pgp->ev[1].x * t23 + pgp->ev[2].x * t31 + pgp->ev[3].x * t0;
    s->s23 = pgp->ev[0].y * t12 + pgp->ev[1].y * t23 + pgp->ev[2].y * t31 + pgp->ev[3].y * t0;
    s->s31 = pgp->ev[0].z * t12 + pgp->ev[1].z * t23 + pgp->ev[2].z * t31 + pgp->ev[3].z * t0;
    s->s0 = pgp->ev[0].w * t12 + pgp->ev[1].w * t23 + pgp->ev[2].w * t31 + pgp->ev[3].w * t0;

    /* debug */
    _cs2_debug_verify_spinor(s);
}
