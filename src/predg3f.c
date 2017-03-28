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
#include "cs2/predg3f.h"
#include "cs2/vec4f.h"
#include "cs2/pin3f.h"
#include "cs2/spinquad3f.h"
#include <math.h>
#include <cs2/assert.h>

#define PI (3.1415926535897932385)
#define EPS (10e-8)

/* some platforms do not define an optimized sincos routine (eg. freebsd) */
#ifndef sincos

static void sincos(double x, double *s, double *c)
{
    *s = sin(x);
    *c = cos(x);
}

#endif /* sincos */

static int almost_zero(double x)
{
    return fabs(x) < EPS;
}

static int almost_zero_vector(const struct cs2_vec3f_s *v)
{
    return almost_zero(v->x) && almost_zero(v->y) && almost_zero(v->z);
}

static int almost_equal(double x, double y)
{
    return almost_zero(x - y);
}

static double pclamp(double x)
{
    if (x < 0.0)
    {
        CS2_ASSERT(x >= -EPS); /* guard nonsense */
        x = 0.0;
    }

    return x;
}

static void calc_r(struct cs2_vec3f_s *r, const struct cs2_vec3f_s *v)
{
    if (!almost_zero(v->x))
        cs2_vec3f_set(r, -v->y, v->x, v->z); /* take x-y plane */
    else if (!almost_zero(v->y))
        cs2_vec3f_set(r, v->x, -v->z, v->y); /* take y-z plane */
    else
        cs2_vec3f_set(r, v->z, v->y, -v->x); /* take z-x plane */
}

static void calc_ellipsoidal_w(struct cs2_vec4f_s *w, const struct cs2_vec3f_s *p, const struct cs2_vec3f_s *q, const struct cs2_vec3f_s *u, const struct cs2_vec3f_s *v, double a, double b)
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
    cs2_pin3f_mul(&ws, &wp, 0.5 / sqrt(pclamp(wp.p0)));
    cs2_vec4f_from_pin3f(w, &ws);
}

static void calc_toroidal_w(struct cs2_vec4f_s *w1, struct cs2_vec4f_s *w2, const struct cs2_vec3f_s *p, const struct cs2_vec3f_s *q, double a)
{
    /*
     * x = (p2^2 [q]^2 - q2^2 [p]^2) u
     * y = (p2^2 [q]^2 - q2^2 [p]^2) v
     * z = (q2 q3 [p]^2 - p2 p3 [q]^2 - l (p2 q3 - p3 q2)) u + (q1 q2 [p]^2 - p1 p2 [q]^2 + l (p1 q2 - p2 q1)) v
     * w = (- q1 q2 [p]^2 - p1 p2 [q]^2 + l (p1 q2 + p2 q1)) u + (q2 q3 [p]^2 + p2 p3 [q]^2 - l (p2 q3 + p3 q2)) v
     *
     * l = a [p] [q]
     * 0 = b [u] [v]
     *
     * w(u, v) = [x, y, z, e]^T
     */
    double pp = cs2_vec3f_sqlen(p);
    double qq = cs2_vec3f_sqlen(q);
    double l = a * sqrt(pp * qq);

    w1->x = p->y * p->y * qq - q->y * q->y * pp;
    w1->y = 0;
    w1->z = q->y * q->z * pp - p->y * p->z * qq - l * (p->y * q->z - p->z * q->y);
    w1->w = - q->x * q->y * pp - p->x * p->y * qq + l * (p->x * q->y + p->y * q->x);

    w2->x = 0;
    w2->y = p->y * p->y * qq - q->y * q->y * pp;
    w2->z = q->x * q->y * pp - p->x * p->y * qq + l * (p->x * q->y - p->y * q->x);
    w2->w = q->y * q->z * pp + p->y * p->z * qq - l * (p->y * q->z + p->z * q->y);

    /* normalize */
    cs2_vec4f_mul(w1, w1, 1.0 / cs2_vec4f_len(w1));
    cs2_vec4f_mul(w2, w2, 1.0 / cs2_vec4f_len(w2));
}

static enum cs2_predgparamtype3f_e inproper_param_case(void)
{
    return cs2_predgparamtype3f_an_empty_set;
}

static enum cs2_predgparamtype3f_e ellipsoidal_param_case(double a, double b, double c)
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
     *  (1) an empty case
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
    if (almost_equal(a, b))
    {
        double t = a + b;

        if (almost_equal(c, -t))
            return cs2_predgparamtype3f_a_pair_of_points;

        if (almost_equal(c, 0))
            return cs2_predgparamtype3f_a_pair_of_yz_crossed_ellipsoids;

        if (almost_equal(c, t))
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
        if (almost_equal(c, - a - b))
            return cs2_predgparamtype3f_a_pair_of_points;

        if (almost_equal(c, a - b))
            return cs2_predgparamtype3f_a_pair_of_y_touching_ellipsoids;

        if (almost_equal(c, b - a))
            return cs2_predgparamtype3f_a_notched_y_barrel;

        if (almost_equal(c, a + b))
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
        if (almost_equal(c, - a - b))
            return cs2_predgparamtype3f_a_pair_of_points;

        if (almost_equal(c, b - a))
            return cs2_predgparamtype3f_a_pair_of_z_touching_ellipsoids;

        if (almost_equal(c, a - b))
            return cs2_predgparamtype3f_a_notched_z_barrel;

        if (almost_equal(c, a + b))
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

    CS2_ASSERT(0);
    return cs2_predgparamtype3f_an_empty_set;
}

static enum cs2_predgparamtype3f_e toroidal_param_case(double a, double b, double c)
{
    if (almost_zero(a) && !almost_zero(b))
    {
        if (c >= -b && c <= b)
            return cs2_predgparamtype3f_a_torus;
        else
            return cs2_predgparamtype3f_an_empty_set;
    }

    if (!almost_zero(a) && almost_zero(b))
    {
        if (c >= -a && c <= a)
            return cs2_predgparamtype3f_a_torus;
        else
            return cs2_predgparamtype3f_an_empty_set;
    }

    CS2_ASSERT(0);
    return cs2_predgparamtype3f_an_empty_set;
}

static void debug_verify_polar_decomposition(struct cs2_mat44f_s *m, const struct cs2_predg3f_s *g)
{
    struct cs2_spinquad3f_s sp;
    int i;

    cs2_spinquad3f_from_predg3f(&sp, g);

    for (i = 0; i < 4; ++i)
    {
        double x = sp.a11 * m->m[0][i] + sp.a12 * m->m[1][i] + sp.a13 * m->m[2][i] + sp.a14 * m->m[3][i];
        double y = sp.a12 * m->m[0][i] + sp.a22 * m->m[1][i] + sp.a23 * m->m[2][i] + sp.a24 * m->m[3][i];
        double z = sp.a13 * m->m[0][i] + sp.a23 * m->m[1][i] + sp.a33 * m->m[2][i] + sp.a34 * m->m[3][i];
        double w = sp.a14 * m->m[0][i] + sp.a24 * m->m[1][i] + sp.a34 * m->m[2][i] + sp.a44 * m->m[3][i];
        double l = sqrt(x * x + y * y + z * z + w * w);
        double ex = m->m[0][i];
        double ey = m->m[1][i];
        double ez = m->m[2][i];
        double ew = m->m[3][i];

        /* normalize */
        x /= l;
        y /= l;
        z /= l;
        w /= l;

        CS2_ASSERT((fabs(x - ex) + fabs(y - ey) + fabs(z - ez) + fabs(w - ew) < EPS) ||
               (fabs(x + ex) + fabs(y + ey) + fabs(z + ez) + fabs(w + ew) < EPS) );
    }
}

static void predgparam3f_eval_an_empty_set(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
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
    CS2_ASSERT(0);
}

static void predgparam3f_eval_a_pair_of_points(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
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
            CS2_ASSERT(0);
            break;
    }
}

static void predgparam3f_eval_a_pair_of_separate_ellipsoids(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
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
            CS2_ASSERT(0);
            break;
    }

    a = u * 2.0 * PI;
    b = v * PI;

    sincos(a, &sa, &ca);
    sincos(b, &sb, &cb);

    *t12 = sqrt(r / (pp->a + pp->b)) * sb * ca;
    *t23 = sqrt(r / pp->a) * sb * sa;
    *t31 = sqrt(r / pp->b) * cb;
    *t0 = sgn * sqrt(pclamp(1.0 - *t12 * *t12 - *t23 * *t23 - *t31 * *t31));
}

static void predgparam3f_eval_a_pair_of_y_touching_ellipsoids(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
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
    CS2_ASSERT(0);
}

static void predgparam3f_eval_a_pair_of_yz_crossed_ellipsoids(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
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
    CS2_ASSERT(0);
}

static void predgparam3f_eval_a_pair_of_z_touching_ellipsoids(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
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
    CS2_ASSERT(0);
}

static void predgparam3f_eval_a_y_barrel(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
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

    a = u * 2 * PI;
    h = 2.0 * v - 1.0;
    sincos(a, &sa, &ca);

    x = sqrt((pp->b - pp->a + pp->c) / (2.0 * pp->b)) * ca;
    z = sqrt((pp->b - pp->a + pp->c) / (2.0 * (pp->b - pp->a))) * sa;
    y = h * sqrt(pclamp(1.0 - x * x - z * z));
    d = sqrt(pclamp((pp->a + pp->b + pp->c) / (2.0 * ((pp->a + pp->b) * x * x + pp->a * y * y + pp->b * z * z))));

    *t12 = x * d;
    *t23 = y * d;
    *t31 = z * d;
    *t0 = sgn * sqrt(pclamp(1.0 - *t12 * *t12 - *t23 * *t23 - *t31 * *t31));
}

static void predgparam3f_eval_a_z_barrel(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
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

    a = u * 2 * PI;
    h = 2.0 * v - 1.0;
    sincos(a, &sa, &ca);

    x = sqrt((pp->a - pp->b + pp->c) / (2.0 * pp->a)) * ca;
    y = sqrt((pp->a - pp->b + pp->c) / (2.0 * (pp->a - pp->b))) * sa;
    z = h * sqrt(pclamp(1.0 - x * x - y * y));
    d = sqrt(pclamp((pp->a + pp->b + pp->c) / (2.0 * ((pp->a + pp->b) * x * x + pp->a * y * y + pp->b * z * z))));

    *t12 = x * d;
    *t23 = y * d;
    *t31 = z * d;
    *t0 = sgn * sqrt(pclamp(1.0 - *t12 * *t12 - *t23 * *t23 - *t31 * *t31));
}

static void predgparam3f_eval_a_notched_y_barrel(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
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
    CS2_ASSERT(0);
}

static void predgparam3f_eval_a_notched_z_barrel(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
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
    CS2_ASSERT(0);
}

static void predgparam3f_eval_a_pair_of_separate_yz_caps(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
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
            CS2_ASSERT(0);
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

    a = u * 2 * PI;
    sincos(a, &sa, &ca);

    y = sqrt((pp->a + pp->b - pp->c) / (2.0 * pp->b)) * ca;
    z = sqrt((pp->a + pp->b - pp->c) / (2.0 * pp->a)) * sa;
    x = side * sqrt(pclamp(1.0 - y * y - z * z));

    x = x * (1 - v) + side * v;
    y = y * (1 - v);
    z = z * (1 - v);

    d = sqrt(pclamp((pp->a + pp->b + pp->c) / (2.0 * ((pp->a + pp->b) * x * x + pp->a * y * y + pp->b * z * z))));

    *t12 = x * d;
    *t23 = y * d;
    *t31 = z * d;
    *t0 = sgn * sqrt(pclamp(1.0 - *t12 * *t12 - *t23 * *t23 - *t31 * *t31));
}

static void predgparam3f_eval_a_torus(double *t12, double *t23, double *t31, double *t0, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{
    double alpha = u * 2 * PI;
    double beta = v * 2 * PI;

    CS2_ASSERT(component == 0);

    if (almost_zero(pp->b))
    {
        double rp = sqrt((pp->a + pp->c) / (2 * pp->a));
        double rm = sqrt((pp->a - pp->c) / (2 * pp->a));

        *t12 = rp * cos(alpha);
        *t23 = rp * sin(alpha);
        *t31 = rm * cos(beta);
        *t0 = rm * sin(beta);
    }
    else
    {
        double rp = sqrt((pp->b + pp->c) / (2 * pp->b));
        double rm = sqrt((pp->b - pp->c) / (2 * pp->b));

        *t12 = rp * cos(alpha);
        *t23 = rm * sin(alpha);
        *t31 = rp * cos(beta);
        *t0 = rm * sin(beta);
    }
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

    calc_r(&r, &h->p.n);
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
    case cs2_predgtype3f_inproper: return "inproper";
    case cs2_predgtype3f_ellipsoidal: return "ellipsoidal";
    case cs2_predgtype3f_toroidal: return "toroidal";
    default: return 0;
    }
}

enum cs2_predgtype3f_e cs2_predg3f_type(const struct cs2_predg3f_s *g)
{
    struct cs2_vec3f_s p, q, u, v;
    double a, b;
    int za, zb;

    cs2_predg3f_pquv(&p, &q, &u, &v, g);

    a = cs2_vec3f_len(&p) * cs2_vec3f_len(&q);
    b = cs2_vec3f_len(&u) * cs2_vec3f_len(&v);
    za = almost_zero(a);
    zb = almost_zero(b);

    if (!za && !zb)
        return cs2_predgtype3f_ellipsoidal;
    else if (!za || !zb)
        return cs2_predgtype3f_toroidal;
    else
        return cs2_predgtype3f_inproper;
}

const char *cs2_predgparamtype3f_str(enum cs2_predgparamtype3f_e pt)
{
    switch (pt)
    {
    /* common */
    case cs2_predgparamtype3f_an_empty_set: return "an empty case";

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
    case cs2_predgparamtype3f_a_torus: return "a torus";

    default: return 0;
    }
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
    case cs2_predgparamtype3f_a_torus: return 2;

    default:
        CS2_ASSERT(0);
        return -2;
    }
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
    case cs2_predgparamtype3f_a_torus: return 1;

    default:
        CS2_ASSERT(0);
        return -1;
    }
}

void cs2_predg3f_param(struct cs2_predgparam3f_s *pp, const struct cs2_predg3f_s *g)
{
    struct cs2_vec3f_s p, q, u, v;
    int za, zb;

    cs2_predg3f_pquv(&p, &q, &u, &v, g);

    pp->a = cs2_vec3f_len(&p) * cs2_vec3f_len(&q);
    pp->b = cs2_vec3f_len(&u) * cs2_vec3f_len(&v);
    pp->c = g->c;

    za = almost_zero(pp->a);
    zb = almost_zero(pp->b);

    if (!za || !zb)
        cs2_predg3f_eigen(&pp->q, 0, g);
    else
        cs2_mat44f_zero(&pp->q);

    if (!za && !zb)
        pp->t = ellipsoidal_param_case(pp->a, pp->b, pp->c);
    else if (!za || !zb)
        pp->t = toroidal_param_case(pp->a, pp->b, pp->c);
    else
        pp->t = inproper_param_case();
}

void cs2_predgparam3f_eval(struct cs2_spin3f_s *s, const struct cs2_predgparam3f_s *pp, double u, double v, int component)
{
    double t12 = 0.0, t23 = 0.0, t31 = 0.0, t0 = 0.0;

    CS2_ASSERT(u >= 0.0 && u <= 1.0 && v >= 0.0 && v <= 1.0);

    switch (pp->t)
    {
    /* common */
    case cs2_predgparamtype3f_an_empty_set:
        predgparam3f_eval_an_empty_set(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    /* ellipsoidal */
    case cs2_predgparamtype3f_a_pair_of_points:
        predgparam3f_eval_a_pair_of_points(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    case cs2_predgparamtype3f_a_pair_of_separate_ellipsoids:
        predgparam3f_eval_a_pair_of_separate_ellipsoids(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    case cs2_predgparamtype3f_a_pair_of_y_touching_ellipsoids:
        predgparam3f_eval_a_pair_of_y_touching_ellipsoids(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    case cs2_predgparamtype3f_a_pair_of_yz_crossed_ellipsoids:
        predgparam3f_eval_a_pair_of_yz_crossed_ellipsoids(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    case cs2_predgparamtype3f_a_pair_of_z_touching_ellipsoids:
        predgparam3f_eval_a_pair_of_z_touching_ellipsoids(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    case cs2_predgparamtype3f_a_y_barrel:
        predgparam3f_eval_a_y_barrel(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    case cs2_predgparamtype3f_a_z_barrel:
        predgparam3f_eval_a_z_barrel(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    case cs2_predgparamtype3f_a_notched_y_barrel:
        predgparam3f_eval_a_notched_y_barrel(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    case cs2_predgparamtype3f_a_notched_z_barrel:
        predgparam3f_eval_a_notched_z_barrel(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    case cs2_predgparamtype3f_a_pair_of_separate_yz_caps:
        predgparam3f_eval_a_pair_of_separate_yz_caps(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    /* toroidal */
    case cs2_predgparamtype3f_a_torus:
        predgparam3f_eval_a_torus(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    default:
        CS2_ASSERT(0);
        break;
    }

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
            case cs2_predgtype3f_inproper:
                cs2_vec4f_set(&w1, 1.0, 0.0, 0.0, 0.0);
                cs2_vec4f_set(&w2, 0.0, 1.0, 0.0, 0.0);
                cs2_vec4f_set(&w3, 0.0, 0.0, 1.0, 0.0);
                cs2_vec4f_set(&w4, 0.0, 0.0, 0.0, 1.0);

                break;

            case cs2_predgtype3f_ellipsoidal:
                calc_ellipsoidal_w(&w1, &p, &q, &u, &v, 1.0, 1.0);
                calc_ellipsoidal_w(&w2, &p, &q, &u, &v, 1.0, -1.0);
                calc_ellipsoidal_w(&w3, &p, &q, &u, &v, -1.0, 1.0);
                calc_ellipsoidal_w(&w4, &p, &q, &u, &v, -1.0, -1.0);

                break;

            case cs2_predgtype3f_toroidal:
                if (almost_zero_vector(&p) || almost_zero_vector(&q))
                {
                    calc_toroidal_w(&w1, &w3, &u, &v, 1.0);
                    calc_toroidal_w(&w2, &w4, &u, &v, -1.0);
                }
                else if (almost_zero_vector(&u) || almost_zero_vector(&v))
                {
                    calc_toroidal_w(&w1, &w2, &p, &q, 1.0);
                    calc_toroidal_w(&w3, &w4, &p, &q, -1.0);
                }
                else
                {
                    CS2_ASSERT(0);
                }

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
        debug_verify_polar_decomposition(m, g);
    }
}
