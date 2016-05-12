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
#include "cs2/predg3f.h"
#include "cs2/vec4f.h"
#include "cs2/spinquad3f.h"
#include <math.h>
#include <assert.h>

#define PI (3.1415926535897932385)
#define EPS (10e-8)

static int almost_zero(double x)
{
    return fabs(x) < EPS;
}

static int almost_zero_vector(const vec3f_t *v)
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
        assert(x >= -EPS); /* guard nonsense */
        x = 0.0;
    }

    return x;
}

static void calc_r(vec3f_t *r, const vec3f_t *v)
{
    if (!almost_zero(v->x))
        vec3f_set(r, -v->y, v->x, v->z); /* take x-y plane */
    else if (!almost_zero(v->y))
        vec3f_set(r, v->x, -v->z, v->y); /* take y-z plane */
    else
        vec3f_set(r, v->z, v->y, -v->x); /* take z-x plane */
}

static void calc_ellipsoidal_w(vec4f_t *w, const vec3f_t *p, const vec3f_t *q, const vec3f_t *u, const vec3f_t *v, double a, double b)
{
    /*
     * d = a [p] [q] (U x V) + b [u] [v] (P x Q) - (P x U) x (Q x V)
     * e = a [p] [q] (U.V) + b [u] [v] (P.Q) - (P x U).(Q x V) - a b [p] [q] [u] [v]
     *
     * w = [d3, d1, d2, e]^T
     */
    double apq = a * vec3f_len(p) * vec3f_len(q);
    double buv = b * vec3f_len(u) * vec3f_len(v);
    vec3f_t pxq, uxv, pxu, qxv, pxuxqxv;

    vec3f_cross(&pxq, p, q);
    vec3f_cross(&uxv, u, v);
    vec3f_cross(&pxu, p, u);
    vec3f_cross(&qxv, q, v);
    vec3f_cross(&pxuxqxv, &pxu, &qxv);

    w->x = apq * uxv.z + buv * pxq.z - pxuxqxv.z;
    w->y = apq * uxv.x + buv * pxq.x - pxuxqxv.x;
    w->z = apq * uxv.y + buv * pxq.y - pxuxqxv.y;
    w->w = apq * vec3f_dot(u, v) + buv * vec3f_dot(p, q) - vec3f_dot(&pxu, &qxv) - apq * buv;
}

static void calc_toroidal_w(vec4f_t *w1, vec4f_t *w2, const vec3f_t *p, const vec3f_t *q, double a)
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
    double pp = vec3f_sqlen(p);
    double qq = vec3f_sqlen(q);
    double l = a * sqrt(pp * qq);

    w1->x = p->y * p->y * qq - q->y * q->y * pp;
    w1->y = 0;
    w1->z = q->y * q->z * pp - p->y * p->z * qq - l * (p->y * q->z - p->z * q->y);
    w1->w = - q->x * q->y * pp - p->x * p->y * qq + l * (p->x * q->y + p->y * q->x);

    w2->x = 0;
    w2->y = p->y * p->y * qq - q->y * q->y * pp;
    w2->z = q->x * q->y * pp - p->x * p->y * qq + l * (p->x * q->y - p->y * q->x);
    w2->w = q->y * q->z * pp + p->y * p->z * qq - l * (p->y * q->z + p->z * q->y);
}

static predgparamtype3f_t inproper_param_case()
{
    return predgparamtype3f_an_empty_set;
}

static predgparamtype3f_t ellipsoidal_param_case(double a, double b, double c)
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
            return predgparamtype3f_a_pair_of_points;

        if (almost_equal(c, 0))
            return predgparamtype3f_a_pair_of_yz_crossed_ellipsoids;

        if (almost_equal(c, t))
            return predgparamtype3f_a_pair_of_separate_yz_caps;

        if (c < -t)
            return predgparamtype3f_an_empty_set;

        if (c > -t && c < 0)
            return predgparamtype3f_a_pair_of_separate_ellipsoids;

        if (c > 0 && c < t)
            return predgparamtype3f_a_pair_of_separate_yz_caps;

        if (c > t)
            return predgparamtype3f_an_empty_set;

        /* fall through */
    }
    /* a < b */
    else if (a < b)
    {
        if (almost_equal(c, - a - b))
            return predgparamtype3f_a_pair_of_points;

        if (almost_equal(c, a - b))
            return predgparamtype3f_a_pair_of_y_touching_ellipsoids;

        if (almost_equal(c, b - a))
            return predgparamtype3f_a_notched_y_barrel;

        if (almost_equal(c, a + b))
            return predgparamtype3f_a_pair_of_separate_yz_caps;

        if (c < - a - b)
            return predgparamtype3f_an_empty_set;

        if (c > - a - b && c < a - b)
            return predgparamtype3f_a_pair_of_separate_ellipsoids;

        if (c > a - b && c < b - a)
            return predgparamtype3f_a_y_barrel;

        if (c > b - a && c < a + b)
            return predgparamtype3f_a_pair_of_separate_yz_caps;

        if (c > a + b)
            return predgparamtype3f_an_empty_set;

        /* fall through */
    }
    /* a > b */
    else if (a > b)
    {
        if (almost_equal(c, - a - b))
            return predgparamtype3f_a_pair_of_points;

        if (almost_equal(c, b - a))
            return predgparamtype3f_a_pair_of_z_touching_ellipsoids;

        if (almost_equal(c, a - b))
            return predgparamtype3f_a_notched_z_barrel;

        if (almost_equal(c, a + b))
            return predgparamtype3f_a_pair_of_separate_yz_caps;

        if (c < - a - b)
            return predgparamtype3f_an_empty_set;

        if (c > - a - b && c < b - a)
            return predgparamtype3f_a_pair_of_separate_ellipsoids;

        if (c > b - a && c < a - b)
            return predgparamtype3f_a_z_barrel;

        if (c > a - b && c < a + b)
            return predgparamtype3f_a_pair_of_separate_yz_caps;

        if (c > a + b)
            return predgparamtype3f_an_empty_set;

        /* fall through */
    }

    assert(0);
    return predgparamtype3f_an_empty_set;
}

static predgparamtype3f_t toroidal_param_case(double a, double b, double c)
{
    if (almost_zero(a) && !almost_zero(b))
    {
        if (c >= -b && c <= b)
            return predgparamtype3f_a_torus;
        else
            return predgparamtype3f_an_empty_set;
    }

    if (!almost_zero(a) && almost_zero(b))
    {
        if (c >= -a && c <= a)
            return predgparamtype3f_a_torus;
        else
            return predgparamtype3f_an_empty_set;
    }

    assert(0);
    return predgparamtype3f_an_empty_set;
}

static void debug_verify_polar_decomposition(mat44f_t *m, const predg3f_t *g)
{
    spinquad3f_t sp;
    int i;

    spinquad3f_from_predg3f(&sp, g);

    for (i = 0; i < 4; ++i)
    {
        double x = sp.a11 * m->m[0][i] + sp.a12 * m->m[1][i] + sp.a13 * m->m[2][i] + sp.a14 * m->m[3][i];
        double y = sp.a12 * m->m[0][i] + sp.a22 * m->m[1][i] + sp.a23 * m->m[2][i] + sp.a24 * m->m[3][i];
        double z = sp.a13 * m->m[0][i] + sp.a23 * m->m[1][i] + sp.a33 * m->m[2][i] + sp.a34 * m->m[3][i];
        double w = sp.a14 * m->m[0][i] + sp.a24 * m->m[1][i] + sp.a34 * m->m[2][i] + sp.a44 * m->m[3][i];
        double l = sqrt(x * x + y * y  + z * z + w * w);
        double ex = m->m[0][i];
        double ey = m->m[1][i];
        double ez = m->m[2][i];
        double ew = m->m[3][i];

        /* normalize */
        x /= l;
        y /= l;
        z /= l;
        w /= l;

        assert((fabs(x - ex) + fabs(y - ey) + fabs(z - ez) + fabs(w - ew) < EPS) ||
               (fabs(x + ex) + fabs(y + ey) + fabs(z + ez) + fabs(w + ew) < EPS) );
    }
}

static void predgparam3f_eval_an_empty_set(double *t12, double *t23, double *t31, double *t0, const predgparam3f_t *pp, double u, double v, int component)
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
    assert(0);
}

static void predgparam3f_eval_a_pair_of_points(double *t12, double *t23, double *t31, double *t0, const predgparam3f_t *pp, double u, double v, int component)
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
            assert(0);
            break;
    }
}

static void predgparam3f_eval_a_pair_of_separate_ellipsoids(double *t12, double *t23, double *t31, double *t0, const predgparam3f_t *pp, double u, double v, int component)
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
            assert(0);
            break;
    }

    a = u * 2.0 * PI;
    b = v * PI;

    sa = sin(a);
    ca = cos(a);
    sb = sin(b);
    cb = cos(b);

    *t12 = sqrt(r / (pp->a + pp->b)) * sb * ca;
    *t23 = sqrt(r / pp->a) * sb * sa;
    *t31 = sqrt(r / pp->b) * cb;
    *t0 = sgn * sqrt(pclamp(1.0 - *t12 * *t12 - *t23 * *t23 - *t31 * *t31));
}

static void predgparam3f_eval_a_pair_of_y_touching_ellipsoids(double *t12, double *t23, double *t31, double *t0, const predgparam3f_t *pp, double u, double v, int component)
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
    assert(0);
}

static void predgparam3f_eval_a_pair_of_yz_crossed_ellipsoids(double *t12, double *t23, double *t31, double *t0, const predgparam3f_t *pp, double u, double v, int component)
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
    assert(0);
}

static void predgparam3f_eval_a_pair_of_z_touching_ellipsoids(double *t12, double *t23, double *t31, double *t0, const predgparam3f_t *pp, double u, double v, int component)
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
    assert(0);
}

static void predgparam3f_eval_a_y_barrel(double *t12, double *t23, double *t31, double *t0, const predgparam3f_t *pp, double u, double v, int component)
{
    double a, h;
    double sa, ca;
    double sgn;
    double x, y, z, d;

    assert(component == 0);

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
    sa = sin(a);
    ca = cos(a);

    x = sqrt((pp->b - pp->a + pp->c) / (2.0 * pp->b)) * ca;
    z = sqrt((pp->b - pp->a + pp->c) / (2.0 * (pp->b - pp->a))) * sa;
    y = h * sqrt(pclamp(1.0 - x * x - z * z));
    d = sqrt(pclamp((pp->a + pp->b + pp->c) / (2.0 * ((pp->a + pp->b) * x * x + pp->a * y * y + pp->b * z * z))));

    *t12 = x * d;
    *t23 = y * d;
    *t31 = z * d;
    *t0 = sgn * sqrt(pclamp(1.0 - *t12 * *t12 - *t23 * *t23 - *t31 * *t31));
}

static void predgparam3f_eval_a_z_barrel(double *t12, double *t23, double *t31, double *t0, const predgparam3f_t *pp, double u, double v, int component)
{
    double a, h;
    double sa, ca;
    double sgn;
    double x, y, z, d;

    assert(component == 0);

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
    sa = sin(a);
    ca = cos(a);

    x = sqrt((pp->a - pp->b + pp->c) / (2.0 * pp->a)) * ca;
    y = sqrt((pp->a - pp->b + pp->c) / (2.0 * (pp->a - pp->b))) * sa;
    z = h * sqrt(pclamp(1.0 - x * x - y * y));
    d = sqrt(pclamp((pp->a + pp->b + pp->c) / (2.0 * ((pp->a + pp->b) * x * x + pp->a * y * y + pp->b * z * z))));

    *t12 = x * d;
    *t23 = y * d;
    *t31 = z * d;
    *t0 = sgn * sqrt(pclamp(1.0 - *t12 * *t12 - *t23 * *t23 - *t31 * *t31));
}

static void predgparam3f_eval_a_notched_y_barrel(double *t12, double *t23, double *t31, double *t0, const predgparam3f_t *pp, double u, double v, int component)
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
    assert(0);
}

static void predgparam3f_eval_a_notched_z_barrel(double *t12, double *t23, double *t31, double *t0, const predgparam3f_t *pp, double u, double v, int component)
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
    assert(0);
}

static void predgparam3f_eval_a_pair_of_separate_yz_caps(double *t12, double *t23, double *t31, double *t0, const predgparam3f_t *pp, double u, double v, int component)
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
            assert(0);
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
    sa = sin(a);
    ca = cos(a);

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

static void predgparam3f_eval_a_torus(double *t12, double *t23, double *t31, double *t0, const predgparam3f_t *pp, double u, double v, int component)
{
    double alpha = u * 2 * PI;
    double beta = v * 2 * PI;

    assert(component == 0);

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

void predg3f_set(predg3f_t *g, const vec3f_t *k, const vec3f_t *l, const vec3f_t *a, const vec3f_t *b, double c)
{
    vec3f_copy(&g->k, k);
    vec3f_copy(&g->l, l);
    vec3f_copy(&g->a, a);
    vec3f_copy(&g->b, b);
    g->c = c;
}

void predg3f_copy(predg3f_t *r, const predg3f_t *g)
{
    vec3f_copy(&r->k, &g->k);
    vec3f_copy(&r->l, &g->l);
    vec3f_copy(&r->a, &g->a);
    vec3f_copy(&r->b, &g->b);
    r->c = g->c;
}

void predg3f_from_predh3f(predg3f_t *g, const predh3f_t *h)
{
    vec3f_t r, nr;

    calc_r(&r, &h->p.n);
    vec3f_cross(&nr, &h->p.n, &r);

    g->k = nr;
    vec3f_cross(&g->l, &h->p.n, &nr);
    g->a = h->b;
    vec3f_neg(&g->b, &h->b);
    g->c = 2 * h->p.d * vec3f_sqlen(&nr);
}

void predg3f_from_preds3f(predg3f_t *g, const preds3f_t *s)
{
    g->k = s->k;
    g->l = s->l;
    g->a = s->a;
    g->b = s->b;
    g->c = 0;
}

void predg3f_pquv(vec3f_t *p, vec3f_t *q, vec3f_t *u, vec3f_t *v, const predg3f_t *g)
{
    vec3f_cross(p, &g->k, &g->l);
    vec3f_sub(q, &g->a, &g->b);
    vec3f_sub(u, &g->k, &g->l);
    vec3f_cross(v, &g->a, &g->b);
}

const char *predgtype3f_str(predgtype3f_t t)
{
    switch (t)
    {
    case predgtype3f_inproper: return "inproper";
    case predgtype3f_ellipsoidal: return "ellipsoidal";
    case predgtype3f_toroidal: return "toroidal";
    default: return 0;
    }
}

predgtype3f_t predg3f_type(const predg3f_t *g)
{
    vec3f_t p, q, u, v;
    double a, b;
    int za, zb;

    predg3f_pquv(&p, &q, &u, &v, g);

    a = vec3f_len(&p) * vec3f_len(&q);
    b = vec3f_len(&u) * vec3f_len(&v);
    za = almost_zero(a);
    zb = almost_zero(b);

    if (!za && !zb)
        return predgtype3f_ellipsoidal;
    else if (!za || !zb)
        return predgtype3f_toroidal;
    else
        return predgtype3f_inproper;
}

const char *predgparamtype3f_str(predgparamtype3f_t pt)
{
    switch (pt)
    {
    /* common */
    case predgparamtype3f_an_empty_set: return "an empty case";

    /* ellipsoidal */
    case predgparamtype3f_a_pair_of_points: return "a pair of points";
    case predgparamtype3f_a_pair_of_separate_ellipsoids: return "a pair of separate ellipsoids";
    case predgparamtype3f_a_pair_of_y_touching_ellipsoids: return "a pair of y-touching ellipsoids";
    case predgparamtype3f_a_pair_of_yz_crossed_ellipsoids: return "a pair of yz-crossed ellipsoids";
    case predgparamtype3f_a_pair_of_z_touching_ellipsoids: return "a pair of z-touching ellipsoids";
    case predgparamtype3f_a_y_barrel: return "a y-barrel";
    case predgparamtype3f_a_z_barrel: return "a z-barrel";
    case predgparamtype3f_a_notched_y_barrel: return "a notched y-barrel";
    case predgparamtype3f_a_notched_z_barrel: return "a notched z-barrel";
    case predgparamtype3f_a_pair_of_separate_yz_caps: return "a pair of separate yz-caps";

    /* toroidal */
    case predgparamtype3f_a_torus: return "a torus";

    default: return 0;
    }
}

int predgparamtype3f_dim(predgparamtype3f_t pt)
{
    switch (pt)
    {
    /* common */
    case predgparamtype3f_an_empty_set: return -1;

    /* ellipsoidal */
    case predgparamtype3f_a_pair_of_points: return 0;
    case predgparamtype3f_a_pair_of_separate_ellipsoids: return 2;
    case predgparamtype3f_a_pair_of_y_touching_ellipsoids: return 2; /* note: not a 2-manifold */
    case predgparamtype3f_a_pair_of_yz_crossed_ellipsoids: return 2; /* note: not a 2-manifold */
    case predgparamtype3f_a_pair_of_z_touching_ellipsoids: return 2; /* note: not a 2-manifold */
    case predgparamtype3f_a_y_barrel: return 2;
    case predgparamtype3f_a_z_barrel: return 2;
    case predgparamtype3f_a_notched_y_barrel: return 2;
    case predgparamtype3f_a_notched_z_barrel: return 2;
    case predgparamtype3f_a_pair_of_separate_yz_caps: return 2;

    /* toroidal */
    case predgparamtype3f_a_torus: return 2;

    default:
        assert(0);
        return -2;
    }
}

int predgparamtype3f_components(predgparamtype3f_t pt)
{
    switch (pt)
    {
    /* common */
    case predgparamtype3f_an_empty_set: return 0;

    /* ellipsoidal */
    case predgparamtype3f_a_pair_of_points: return 2;
    case predgparamtype3f_a_pair_of_separate_ellipsoids: return 2;
    case predgparamtype3f_a_pair_of_y_touching_ellipsoids: return 1;
    case predgparamtype3f_a_pair_of_yz_crossed_ellipsoids: return 1;
    case predgparamtype3f_a_pair_of_z_touching_ellipsoids: return 1;
    case predgparamtype3f_a_y_barrel: return 1;
    case predgparamtype3f_a_z_barrel: return 1;
    case predgparamtype3f_a_notched_y_barrel: return 1;
    case predgparamtype3f_a_notched_z_barrel: return 1;
    case predgparamtype3f_a_pair_of_separate_yz_caps: return 2;

    /* toroidal */
    case predgparamtype3f_a_torus: return 1;

    default:
        assert(0);
        return -1;
    }
}

void predg3f_param(predgparam3f_t *pp, const predg3f_t *g)
{
    vec3f_t p, q, u, v;
    int za, zb;

    predg3f_pquv(&p, &q, &u, &v, g);

    pp->a = vec3f_len(&p) * vec3f_len(&q);
    pp->b = vec3f_len(&u) * vec3f_len(&v);
    pp->c = g->c;

    za = almost_zero(pp->a);
    zb = almost_zero(pp->b);

    if (!za || !zb)
        predg3f_eigen(&pp->q, 0, g);
    else
        mat44f_zero(&pp->q);

    if (!za && !zb)
        pp->t = ellipsoidal_param_case(pp->a, pp->b, pp->c);
    else if (!za || !zb)
        pp->t = toroidal_param_case(pp->a, pp->b, pp->c);
    else
        pp->t = inproper_param_case();
}

void predgparam3f_eval(spin3f_t *s, const predgparam3f_t *pp, double u, double v, int component)
{
    double t12 = 0.0, t23 = 0.0, t31 = 0.0, t0 = 0.0;

    assert(u >= 0.0 && u <= 1.0 && v >= 0.0 && v <= 1.0);

    switch (pp->t)
    {
    /* common */
    case predgparamtype3f_an_empty_set:
        predgparam3f_eval_an_empty_set(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    /* ellipsoidal */
    case predgparamtype3f_a_pair_of_points:
        predgparam3f_eval_a_pair_of_points(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    case predgparamtype3f_a_pair_of_separate_ellipsoids:
        predgparam3f_eval_a_pair_of_separate_ellipsoids(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    case predgparamtype3f_a_pair_of_y_touching_ellipsoids:
        predgparam3f_eval_a_pair_of_y_touching_ellipsoids(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    case predgparamtype3f_a_pair_of_yz_crossed_ellipsoids:
        predgparam3f_eval_a_pair_of_yz_crossed_ellipsoids(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    case predgparamtype3f_a_pair_of_z_touching_ellipsoids:
        predgparam3f_eval_a_pair_of_z_touching_ellipsoids(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    case predgparamtype3f_a_y_barrel:
        predgparam3f_eval_a_y_barrel(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    case predgparamtype3f_a_z_barrel:
        predgparam3f_eval_a_z_barrel(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    case predgparamtype3f_a_notched_y_barrel:
        predgparam3f_eval_a_notched_y_barrel(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    case predgparamtype3f_a_notched_z_barrel:
        predgparam3f_eval_a_notched_z_barrel(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    case predgparamtype3f_a_pair_of_separate_yz_caps:
        predgparam3f_eval_a_pair_of_separate_yz_caps(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    /* toroidal */
    case predgparamtype3f_a_torus:
        predgparam3f_eval_a_torus(&t12, &t23, &t31, &t0, pp, u, v, component);
        break;

    default:
        assert(0);
        break;
    }

    /* eigenmatrix rotation */
    s->s12 = pp->q.m[0][0] * t12 + pp->q.m[0][1] * t23 + pp->q.m[0][2] * t31 + pp->q.m[0][3] * t0;
    s->s23 = pp->q.m[1][0] * t12 + pp->q.m[1][1] * t23 + pp->q.m[1][2] * t31 + pp->q.m[1][3] * t0;
    s->s31 = pp->q.m[2][0] * t12 + pp->q.m[2][1] * t23 + pp->q.m[2][2] * t31 + pp->q.m[2][3] * t0;
    s->s0 = pp->q.m[3][0] * t12 + pp->q.m[3][1] * t23 + pp->q.m[3][2] * t31 + pp->q.m[3][3] * t0;
}

void predg3f_eigen(mat44f_t *m, vec4f_t *e, const predg3f_t *g)
{
    vec3f_t p, q, u, v;
    vec4f_t w1, w2, w3, w4;
    double pl, ql, ul, vl, w1l, w2l, w3l, w4l;

    /* base */
    predg3f_pquv(&p, &q, &u, &v, g);

    /* eigenvalues */
    if (e)
    {
        pl = vec3f_len(&p);
        ql = vec3f_len(&q);
        ul = vec3f_len(&u);
        vl = vec3f_len(&v);

        e->x = g->c - (pl * ql + ul * vl);
        e->y = g->c - (pl * ql - ul * vl);
        e->z = g->c - (- pl * ql + ul * vl);
        e->w = g->c - (- pl * ql - ul * vl);
    }

    /* eigenvectors */
    if (m)
    {
        /* eigenvectors */
        switch (predg3f_type(g))
        {
            case predgtype3f_inproper:
                vec4f_set(&w1, 1.0, 0.0, 0.0, 0.0);
                vec4f_set(&w2, 0.0, 1.0, 0.0, 0.0);
                vec4f_set(&w3, 0.0, 0.0, 1.0, 0.0);
                vec4f_set(&w4, 0.0, 0.0, 0.0, 1.0);

                break;

            case predgtype3f_ellipsoidal:
                calc_ellipsoidal_w(&w1, &p, &q, &u, &v, 1.0, 1.0);
                calc_ellipsoidal_w(&w2, &p, &q, &u, &v, 1.0, -1.0);
                calc_ellipsoidal_w(&w3, &p, &q, &u, &v, -1.0, 1.0);
                calc_ellipsoidal_w(&w4, &p, &q, &u, &v, -1.0, -1.0);

                break;

            case predgtype3f_toroidal:
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
                    assert(0);
                }

                break;
        }

        /* normalize */
        w1l = vec4f_len(&w1);
        w2l = vec4f_len(&w2);
        w3l = vec4f_len(&w3);
        w4l = vec4f_len(&w4);

        m->m[0][0] = w1.x / w1l;
        m->m[1][0] = w1.y / w1l;
        m->m[2][0] = w1.z / w1l;
        m->m[3][0] = w1.w / w1l;

        m->m[0][1] = w2.x / w2l;
        m->m[1][1] = w2.y / w2l;
        m->m[2][1] = w2.z / w2l;
        m->m[3][1] = w2.w / w2l;

        m->m[0][2] = w3.x / w3l;
        m->m[1][2] = w3.y / w3l;
        m->m[2][2] = w3.z / w3l;
        m->m[3][2] = w3.w / w3l;

        m->m[0][3] = w4.x / w4l;
        m->m[1][3] = w4.y / w4l;
        m->m[2][3] = w4.z / w4l;
        m->m[3][3] = w4.w / w4l;

        /* debug */
        debug_verify_polar_decomposition(m, g);
    }
}
