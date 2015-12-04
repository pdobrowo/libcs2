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

static int almost_zero(const vec3f_t *v)
{
    static const double EPS = 10e-8;

    return fabs(v->x) < EPS && fabs(v->y) < EPS && fabs(v->z) < EPS;
}

static void choose_r_vector(vec3f_t *r, const vec3f_t *v)
{
    if (v->x != 0)
        vec3f_set(r, -v->y, v->x, v->z); /* take x-y plane */
    else if (v->y != 0)
        vec3f_set(r, v->x, -v->z, v->y); /* take y-z plane */
    else
        vec3f_set(r, v->z, v->y, -v->x); /* take z-x plane */
}

static void calc_pquv(vec3f_t *p, vec3f_t *q, vec3f_t *u, vec3f_t *v, const predg3f_t *g)
{
    vec3f_cross(p, &g->k, &g->l);
    vec3f_sub(q, &g->a, &g->b);
    vec3f_sub(u, &g->k, &g->l);
    vec3f_cross(v, &g->a, &g->b);
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
    vec3f_mul(&d, &d, pq + uv + l);

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

    vec3f_mad5(&t, &j, pq + uv + l, p, - trq, q, - trp, u, - trv, v, - tru);
    vec3f_mul(&t, &t, l * l - pp * qq - uu * vv);
    vec3f_add(&d, &d, &t);

    /* calc w */
    w->x = d.z;
    w->y = d.x;
    w->z = d.y;

    vec3f_mad4(&t, p, trq, q, trp, u, trv, v, tru);

    w->w = - vec3f_dot(&r, &d) / (pq + uv + l);

    /* foolish way: q->w = l * (vec3f_dot(&t, &r) - 2 * (a * sqrt(uu) * sqrt(vv) * vec3f_tr(&pxq) + b * sqrt(pp) * sqrt(qq) * vec3f_tr(&uxv))) */
}

void predg3f_from_predh3f(predg3f_t *g, const predh3f_t *h)
{
    vec3f_t r, nr;

    choose_r_vector(&r, &h->p.n);
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

predgtype3f_t predg3f_type(const predg3f_t *g)
{
    vec3f_t p, q, u, v;
    int pqf, uvf;

    calc_pquv(&p, &q, &u, &v, g);

    pqf = !almost_zero(&p) && !almost_zero(&q);
    uvf = !almost_zero(&u) && !almost_zero(&v);

    if (pqf && uvf)
        return predgtype3f_proper_ellipsoidal;
    else if (pqf || uvf)
        return predgtype3f_proper_cylindrical;
    else
        return predgtype3f_inproper;
}

void predg3f_param(predgparam3f_t *pp, const predg3f_t *g)
{
    vec3f_t p, q, u, v;
    double pl, ql, ul, vl, su;

    calc_pquv(&p, &q, &u, &v, g);

    pl = vec3f_len(&p);
    ql = vec3f_len(&q);
    ul = vec3f_len(&u);
    vl = vec3f_len(&v);
    su = pl * ql + ul * vl;

    /* radii */
    pp->r12 = sqrt((g->c + su) / (2 * su));
    pp->r23 = sqrt((g->c + su) / (2 * pl * ql));
    pp->r31 = sqrt((g->c + su) / (2 * ul * vl));

    /* matrix q */
    predg3f_eigen(&pp->q, 0, g);
}

void predgparam3f_eval(spin3f_t *s, const predgparam3f_t *pp, double u, double v, double sgn)
{
    double a = u * PI;
    double b = v * 2 * PI;
    double t12 = pp->r12 * sin(a) * cos(b);
    double t23 = pp->r23 * sin(a) * sin(b);
    double t31 = pp->r31 * cos(a);
    double t0 = sgn * sqrt(1 - t12 * t12 - t23 * t23 - t31 * t31);

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

    calc_pquv(&p, &q, &u, &v, g);

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
        assert(predg3f_type(g) == predgtype3f_proper_ellipsoidal);

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
