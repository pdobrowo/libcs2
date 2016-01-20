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
#include "cs2/predg3x.h"
#include <assert.h>

static void calc_r(vec3x_t *r, const vec3x_t *v)
{
    if (mpz_sgn(v->x))
    {
        /* take x-y plane */
        vec3x_set(r, v->y, v->x, v->z);
        mpz_neg(r->x, r->x);
    }
    else if (mpz_sgn(v->y))
    {
        /* take y-z plane */
        vec3x_set(r, v->x, v->z, v->y);
        mpz_neg(r->y, r->y);
    }
    else
    {
        /* take z-x plane */
        vec3x_set(r, v->z, v->y, v->x);
        mpz_neg(r->z, r->z);
    }
}

#if 0

static void calc_w(vec4f_t *w, const vec3x_t *p, const vec3x_t *q, const vec3x_t *u, const vec3x_t *v, double a, double b)
{
    double pp = vec3x_sqlen(p);
    double qq = vec3x_sqlen(q);
    double uu = vec3x_sqlen(u);
    double vv = vec3x_sqlen(v);
    double pq = vec3x_dot(p, q);
    double pv = vec3x_dot(p, v);
    double qu = vec3x_dot(q, u);
    double uv = vec3x_dot(u, v);
    double trp = vec3x_tr(p);
    double trq = vec3x_tr(q);
    double tru = vec3x_tr(u);
    double trv = vec3x_tr(v);
    double l = a * sqrt(pp) * sqrt(qq) + b * sqrt(uu) * sqrt(vv);

    vec3x_t pxq, uxv, pxu, qxv, t, r, d;
    vec3x_t j = { 1, 1, 1 };

    vec3x_cross(&pxq, p, q);
    vec3x_cross(&uxv, u, v);
    vec3x_cross(&pxu, p, u);
    vec3x_cross(&qxv, q, v);

    vec3x_add(&r, &pxq, &uxv);

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
    vec3x_mad2(&d, &pxu, vec3x_tr(&qxv), &qxv, vec3x_tr(&pxu));
    vec3x_mul(&d, &d, pq + uv + l);

    vec3x_mad2(&t, p, tru, u, trp);
    vec3x_mul(&t, &t, - pv * qq - qu * vv);
    vec3x_add(&d, &d, &t);

    vec3x_mad2(&t, q, trv, v, trq);
    vec3x_mul(&t, &t, - pv * uu - qu * pp);
    vec3x_add(&d, &d, &t);

    vec3x_mad2(&t, p, trq, q, trp);
    vec3x_mul(&t, &t, - uu * vv - vec3x_dot(&pxu, &qxv));
    vec3x_add(&d, &d, &t);

    vec3x_mad2(&t, u, trv, v, tru);
    vec3x_mul(&t, &t, - pp * qq - vec3x_dot(&pxu, &qxv));
    vec3x_add(&d, &d, &t);

    vec3x_mad2(&t, p, trp * qq, q, trq * pp);
    vec3x_mul(&t, &t, 2 * uv);
    vec3x_add(&d, &d, &t);

    vec3x_mad2(&t, u, tru * vv, v, trv * uu);
    vec3x_mul(&t, &t, 2 * pq);
    vec3x_add(&d, &d, &t);

    vec3x_mad4(&t, u, trv * pq, v, tru * pq, p, trq * uv, q, trp * uv);
    vec3x_mul(&t, &t, -l);
    vec3x_add(&d, &d, &t);

    vec3x_mad4(&t, q, tru * pv, u, trq * pv, p, trv * qu, v, trp * qu);
    vec3x_mul(&t, &t, l);
    vec3x_add(&d, &d, &t);

    vec3x_mad5(&t, &j, pq + uv + l, p, - trq, q, - trp, u, - trv, v, - tru);
    vec3x_mul(&t, &t, l * l - pp * qq - uu * vv);
    vec3x_add(&d, &d, &t);

    /* calc w */
    w->x = d.z;
    w->y = d.x;
    w->z = d.y;

    vec3x_mad4(&t, p, trq, q, trp, u, trv, v, tru);

    w->w = - vec3x_dot(&r, &d) / (pq + uv + l);
}

#endif

void predg3x_init(predg3x_t *g)
{
    vec3x_init(&g->k);
    vec3x_init(&g->l);
    vec3x_init(&g->a);
    vec3x_init(&g->b);
    mpz_init(g->c);
}

void predg3x_clear(predg3x_t *g)
{
    vec3x_clear(&g->k);
    vec3x_clear(&g->l);
    vec3x_clear(&g->a);
    vec3x_clear(&g->b);
    mpz_clear(g->c);
}

void predg3x_from_predh3x(predg3x_t *g, const predh3x_t *h)
{
    calc_r(&g->l, &h->p.n);
    vec3x_cross(&g->k, &h->p.n, &g->l);
    vec3x_cross(&g->l, &h->p.n, &g->k);
    vec3x_copy(&g->a, &h->b);
    vec3x_neg(&g->b, &h->b);
    vec3x_sqlen(g->c, &g->k);
    mpz_mul(g->c, g->c, h->p.d);
    mpz_mul_2exp(g->c, g->c, 1);
}

void predg3x_from_preds3x(predg3x_t *g, const preds3x_t *s)
{
    vec3x_copy(&g->k, &s->k);
    vec3x_copy(&g->l, &s->l);
    vec3x_copy(&g->a, &s->a);
    vec3x_copy(&g->b, &s->b);
    mpz_set_si(g->c, 0);
}

void predg3x_pquv(vec3x_t *p, vec3x_t *q, vec3x_t *u, vec3x_t *v, const predg3x_t *g)
{
    vec3x_cross(p, &g->k, &g->l);
    vec3x_sub(q, &g->a, &g->b);
    vec3x_sub(u, &g->k, &g->l);
    vec3x_cross(v, &g->a, &g->b);
}

const char *predgtype3x_str(predgtype3x_t t)
{
    switch (t)
    {
    case predgtype3x_inproper: return "inproper";
    case predgtype3x_ellipsoidal: return "ellipsoidal";
    case predgtype3x_toroidal: return "toroidal";
    default: return 0;
    }
}

predgtype3x_t predg3x_type(const predg3x_t *g)
{
    vec3x_t p, q, u, v;
    int pq, uv;
    predgtype3x_t t;
    vec3x_init(&p);
    vec3x_init(&q);
    vec3x_init(&u);
    vec3x_init(&v);
    predg3x_pquv(&p, &q, &u, &v, g);
    pq = !vec3x_is_zero(&p) && !vec3x_is_zero(&q);
    uv = !vec3x_is_zero(&u) && !vec3x_is_zero(&v);
    if (pq && uv)
        t = predgtype3x_ellipsoidal;
    else if (pq || uv)
        t = predgtype3x_toroidal;
    else
        t = predgtype3x_inproper;
    vec3x_clear(&p);
    vec3x_clear(&q);
    vec3x_clear(&u);
    vec3x_clear(&v);
    return t;
}

#if 0

void predg3x_param(predgparam3x_t *pp, const predg3x_t *g)
{
    vec3x_t p, q, u, v;
    double pl, ql, ul, vl, su;

    predg3x_pquv(&p, &q, &u, &v, g);

    pl = vec3x_len(&p);
    ql = vec3x_len(&q);
    ul = vec3x_len(&u);
    vl = vec3x_len(&v);
    su = pl * ql + ul * vl;

    /* radii */
    pp->r12 = sqrt((g->c + su) / (2 * su));
    pp->r23 = sqrt((g->c + su) / (2 * pl * ql));
    pp->r31 = sqrt((g->c + su) / (2 * ul * vl));

    /* matrix q */
    predg3x_eigen(&pp->q, 0, g);
}

void predgparam3x_eval(spin3x_t *s, const predgparam3x_t *pp, double u, double v, double sgn)
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

void predg3x_eigen(mat44f_t *m, vec4f_t *e, const predg3x_t *g)
{
    vec3x_t p, q, u, v;
    vec4f_t w1, w2, w3, w4;
    double pl, ql, ul, vl, w1l, w2l, w3l, w4l;

    predg3x_pquv(&p, &q, &u, &v, g);

    /* eigenvalues */
    if (e)
    {
        pl = vec3x_len(&p);
        ql = vec3x_len(&q);
        ul = vec3x_len(&u);
        vl = vec3x_len(&v);

        e->x = g->c - (pl * ql + ul * vl);
        e->y = g->c - (pl * ql - ul * vl);
        e->z = g->c - (- pl * ql + ul * vl);
        e->w = g->c - (- pl * ql - ul * vl);
    }

    /* eigenvectors */
    if (m)
    {
        /* only ellipsoidal param for now */
        /*assert(predg3x_type(g) == predgtype3x_ellipsoidal);*/

        if (predg3x_type(g) != predgtype3x_ellipsoidal)
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

#endif
