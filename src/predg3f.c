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
#include <math.h>
#include <assert.h>

#define PI (3.1415926535897932385)
#define EPS (10e-8)

static int almost_zero(double x)
{
    return fabs(x) < EPS;
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

static void calc_w(vec4f_t *w, const vec3f_t *p, const vec3f_t *q, const vec3f_t *u, const vec3f_t *v, double a, double b)
{
    double pp = vec3f_sqlen(p);
    double qq = vec3f_sqlen(q);
    double uu = vec3f_sqlen(u);
    double vv = vec3f_sqlen(v);
    double pq = vec3f_dot(p, q);
    double pv = vec3f_dot(p, v);
    double qu = vec3f_dot(q, u);
    double uv = vec3f_dot(u, v);
    double trp = vec3f_tr(p);
    double trq = vec3f_tr(q);
    double tru = vec3f_tr(u);
    double trv = vec3f_tr(v);
    double l = a * sqrt(pp) * sqrt(qq) + b * sqrt(uu) * sqrt(vv);
    double dd = pq + uv + l;

    vec3f_t pxq, uxv, pxu, qxv, t, r, d;
    vec3f_t j = { 1, 1, 1 };

    vec3f_cross(&pxq, p, q);
    vec3f_cross(&uxv, u, v);
    vec3f_cross(&pxu, p, u);
    vec3f_cross(&qxv, q, v);

    vec3f_add(&r, &pxq, &uxv);

    /* calc d:
     *
     * d = + (P.Q + U.V + L) ((Q x V) tr(P x U) + (P x U) tr(Q x V))
     *     - (P tr(U) + U tr(P)) (P.V [Q]^2 + Q.U [V]^2)
     *     - (Q tr(V) + V tr(Q)) (P.V [U]^2 + Q.U [P]^2)
     *     - (P tr(Q) + Q tr(P)) ([U]^2 [V]^2 + (P x U).(Q x V))
     *     - (U tr(V) + V tr(U)) ([P]^2 [Q]^2 + (P x U).(Q x V))
     *     + 2 U.V (P tr(P) [Q]^2 + Q tr(Q) [P]^2)
     *     + 2 P.Q (U tr(U) [V]^2 + V tr(V) [U]^2)
     *     - L (P.Q (U tr(V) + V tr(U)) + U.V (P tr(Q) + Q tr(P)))
     *     + L (P.V (Q tr(U) + U tr(Q)) + Q.U (P tr(V) + V tr(P)))
     *     + (L^2 - [P]^2 [Q]^2 - [U]^2 [V]^2) ( J (P.Q + U.V + L) - (P tr(Q) + Q tr(P) + U tr(V) + V tr(U)) )
     */
    vec3f_mad2(&d, &pxu, vec3f_tr(&qxv), &qxv, vec3f_tr(&pxu));
    vec3f_mul(&d, &d, dd);

    vec3f_mad2(&t, p, tru, u, trp);
    vec3f_mul(&t, &t, - pv * qq - qu * vv);
    vec3f_add(&d, &d, &t);

    vec3f_mad2(&t, q, trv, v, trq);
    vec3f_mul(&t, &t, - pv * uu - qu * pp);
    vec3f_add(&d, &d, &t);

    vec3f_mad2(&t, p, trq, q, trp);
    vec3f_mul(&t, &t, - uu * vv - vec3f_dot(&pxu, &qxv));
    vec3f_add(&d, &d, &t);

    vec3f_mad2(&t, u, trv, v, tru);
    vec3f_mul(&t, &t, - pp * qq - vec3f_dot(&pxu, &qxv));
    vec3f_add(&d, &d, &t);

    vec3f_mad2(&t, p, trp * qq, q, trq * pp);
    vec3f_mul(&t, &t, 2 * uv);
    vec3f_add(&d, &d, &t);

    vec3f_mad2(&t, u, tru * vv, v, trv * uu);
    vec3f_mul(&t, &t, 2 * pq);
    vec3f_add(&d, &d, &t);

    vec3f_mad4(&t, u, trv * pq, v, tru * pq, p, trq * uv, q, trp * uv);
    vec3f_mul(&t, &t, -l);
    vec3f_add(&d, &d, &t);

    vec3f_mad4(&t, q, tru * pv, u, trq * pv, p, trv * qu, v, trp * qu);
    vec3f_mul(&t, &t, l);
    vec3f_add(&d, &d, &t);

    vec3f_mad5(&t, &j, dd, p, - trq, q, - trp, u, - trv, v, - tru);
    vec3f_mul(&t, &t, l * l - pp * qq - uu * vv);
    vec3f_add(&d, &d, &t);

    /* calc w */
    w->x = dd * d.z;
    w->y = dd * d.x;
    w->z = dd * d.y;

    vec3f_mad4(&t, p, trq, q, trp, u, trv, v, tru);

    w->w = -vec3f_dot(&r, &d);
}

static predgparamtype3f_t inproper_param_case()
{
    return predgparamtype3f_empty;
}

static predgparamtype3f_t ellipsoidal_param_case(double a, double b, double c)
{
    if (almost_zero(c + a + b))
        return predgparamtype3f_two_points;

    if (c < - a - b)
        return predgparamtype3f_empty;

    if (a <= b)
    {
        if (c <= a - b)
            return predgparamtype3f_ellipsoid;

        if (c <= b - a)
            return predgparamtype3f_barrel;
    }
    else
    {
        if (c <= b - a)
            return predgparamtype3f_ellipsoid;

        if (c <= a - b)
            return predgparamtype3f_barrel;
    }

    if (c <= a + b)
        return predgparamtype3f_two_caps;

    return predgparamtype3f_empty;
}

static predgparamtype3f_t toroidal_param_case(double a, double b, double c)
{
    if (almost_zero(a))
    {
        if (c >= -b && c <= b)
            return predgparamtype3f_torus;
        else
            return predgparamtype3f_empty;
    }
    else
    {
        if (c >= -a && c <= a)
            return predgparamtype3f_torus;
        else
            return predgparamtype3f_empty;
    }
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
    case predgparamtype3f_empty: return "empty";
    case predgparamtype3f_two_points: return "two_points";
    case predgparamtype3f_ellipsoid: return "ellipsoid";
    case predgparamtype3f_barrel: return "barrel";
    case predgparamtype3f_two_caps: return "two_caps";
    case predgparamtype3f_torus: return "torus";
    default: return 0;
    }
}

int predgparamtype3f_dim(predgparamtype3f_t pt)
{
    switch (pt)
    {
    case predgparamtype3f_empty: return -1;
    case predgparamtype3f_two_points: return 0;
    case predgparamtype3f_ellipsoid: return 2;
    case predgparamtype3f_barrel: return 2;
    case predgparamtype3f_two_caps: return 2;
    case predgparamtype3f_torus: return 2;
    default:
        assert(0);
        return -2;
    }
}

int predgparamtype3f_components(predgparamtype3f_t pt)
{
    switch (pt)
    {
    case predgparamtype3f_empty: return 0;
    case predgparamtype3f_two_points: return 2;
    case predgparamtype3f_ellipsoid: return 2;
    case predgparamtype3f_barrel: return 1;
    case predgparamtype3f_two_caps: return 2;
    case predgparamtype3f_torus: return 1;
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
        case predgparamtype3f_empty:
        {
            /* -1-dimensional case, no parametrization */
            assert(0);
        }
        break;

        case predgparamtype3f_two_points:
        {
            switch (component)
            {
                case 0:
                    t12 = 0.0;
                    t23 = 0.0;
                    t31 = 0.0;
                    t0 = 1.0;
                    break;

                case 1:
                    t12 = 0.0;
                    t23 = 0.0;
                    t31 = 0.0;
                    t0 = -1.0;
                    break;

                default:
                    assert(0);
                    break;
            }
        }
        break;

        case predgparamtype3f_ellipsoid:
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

            t12 = sqrt(r / (pp->a + pp->b)) * sb * ca;
            t23 = sqrt(r / pp->a) * sb * sa;
            t31 = sqrt(r / pp->b) * cb;
            t0 = sgn * sqrt(pclamp(1.0 - t12 * t12 - t23 * t23 - t31 * t31));
        }
        break;

        case predgparamtype3f_barrel:
        {
            /* note: branch cut*/
            double a, h;
            double sa, ca;
            double sgn;

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

            if (pp->c > pp->a - pp->b && pp->c <= pp->b - pp->a)
            {
                /* y-barrel*/
                double x = sqrt((pp->b - pp->a + pp->c) / (2.0 * pp->b)) * ca;
                double z = sqrt((pp->b - pp->a + pp->c) / (2.0 * (pp->b - pp->a))) * sa;
                double y = h * sqrt(pclamp(1.0 - x * x - z * z));
                double d = sqrt(pclamp((pp->a + pp->b + pp->c) / (2.0 * ((pp->a + pp->b) * x * x + pp->a * y * y + pp->b * z * z))));

                t12 = x * d;
                t23 = y * d;
                t31 = z * d;
                t0 = sgn * sqrt(pclamp(1.0 - t12 * t12 - t23 * t23 - t31 * t31));
            }
            else if (pp->c > pp->b - pp->a && pp->c <= pp->a - pp->b)
            {
                 /* z-barrel */
                double x = sqrt((pp->a - pp->b + pp->c) / (2.0 * pp->a)) * ca;
                double y = sqrt((pp->a - pp->b + pp->c) / (2.0 * (pp->a - pp->b))) * sa;
                double z = h * sqrt(pclamp(1.0 - x * x - y * y));
                double d = sqrt(pclamp((pp->a + pp->b + pp->c) / (2.0 * ((pp->a + pp->b) * x * x + pp->a * y * y + pp->b * z * z))));

                t12 = x * d;
                t23 = y * d;
                t31 = z * d;
                t0 = sgn * sqrt(pclamp(1.0 - t12 * t12 - t23 * t23 - t31 * t31));
            }
            else
            {
                /* fail */
                assert(0);
            }
        }
        break;

        case predgparamtype3f_two_caps:
        {
            /* yz-caps */
            double a, x, y, z, d;
            double sa, ca;
            double sgn, side;

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

            t12 = x * d;
            t23 = y * d;
            t31 = z * d;
            t0 = sgn * sqrt(pclamp(1.0 - t12 * t12 - t23 * t23 - t31 * t31));
        }
        break;

        case predgparamtype3f_torus:
        {
            double alpha = u * 2 * PI;
            double beta = v * 2 * PI;

            if (almost_zero(pp->b))
            {
                double rp = sqrt((pp->a + pp->c) / (2 * pp->a));
                double rm = sqrt((pp->a - pp->c) / (2 * pp->a));

                t12 = rp * cos(alpha);
                t23 = rp * sin(alpha);
                t31 = rm * cos(beta);
                t0 = rm * sin(beta);
            }
            else
            {
                double rp = sqrt((pp->b + pp->c) / (2 * pp->b));
                double rm = sqrt((pp->b - pp->c) / (2 * pp->b));

                t12 = rp * cos(alpha);
                t23 = rm * sin(alpha);
                t31 = rp * cos(beta);
                t0 = rm * sin(beta);
            }

            /* debug, currently Q is unknown for toroidal case */
            s->s12 = t12;
            s->s23 = t23;
            s->s31 = t31;
            s->s0 = t0;
            return;
        }
        break;

        default:
            assert(0);
    }

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
        /* only ellipsoidal param for now */

        /*assert(predg3f_type(g) == predgtype3f_ellipsoidal);*/

        if (predg3f_type(g) != predgtype3f_ellipsoidal)
        {
            /* note: debug code */
            m->m[0][0] = 0;
            m->m[1][0] = 0;
            m->m[2][0] = 0;
            m->m[3][0] = 0;

            m->m[0][1] = 0;
            m->m[1][1] = 0;
            m->m[2][1] = 0;
            m->m[3][1] = 0;

            m->m[0][2] = 0;
            m->m[1][2] = 0;
            m->m[2][2] = 0;
            m->m[3][2] = 0;

            m->m[0][3] = 0;
            m->m[1][3] = 0;
            m->m[2][3] = 0;
            m->m[3][3] = 0;

            return;
        }

        calc_w(&w1, &p, &q, &u, &v, 1, 1);
        calc_w(&w2, &p, &q, &u, &v, 1, -1);
        calc_w(&w3, &p, &q, &u, &v, -1, 1);
        calc_w(&w4, &p, &q, &u, &v, -1, -1);

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
    }
}
