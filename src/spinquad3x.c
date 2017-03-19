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
#include "cs2/spinquad3x.h"
#include "cs2/vec3x.h"

void spinquad3x_init(struct spinquad3x_s *sq)
{
    mpz_init(sq->a11);
    mpz_init(sq->a22);
    mpz_init(sq->a33);
    mpz_init(sq->a44);
    mpz_init(sq->a12);
    mpz_init(sq->a13);
    mpz_init(sq->a14);
    mpz_init(sq->a23);
    mpz_init(sq->a24);
    mpz_init(sq->a34);
}

void spinquad3x_clear(struct spinquad3x_s *sq)
{
    mpz_clear(sq->a11);
    mpz_clear(sq->a22);
    mpz_clear(sq->a33);
    mpz_clear(sq->a44);
    mpz_clear(sq->a12);
    mpz_clear(sq->a13);
    mpz_clear(sq->a14);
    mpz_clear(sq->a23);
    mpz_clear(sq->a24);
    mpz_clear(sq->a34);
}

void spinquad3x_from_predh3x(struct spinquad3x_s *sq, const struct predh3x_s *ph)
{
    struct predg3x_s g;
    predg3x_from_predh3x(&g, ph);
    spinquad3x_from_predg3x(sq, &g);
}

void spinquad3x_from_preds3x(struct spinquad3x_s *sq, const struct preds3x_s *ps)
{
    struct predg3x_s g;
    predg3x_from_preds3x(&g, ps);
    spinquad3x_from_predg3x(sq, &g);
}

void spinquad3x_from_predg3x(struct spinquad3x_s *sq, const struct predg3x_s *pg)
{
    struct vec3x_s p, q, u, v;
    mpz_t pxqx, pxqy, pxqz, pyqx, pyqy, pyqz, pzqx, pzqy, pzqz, uxvx, uxvy, uxvz, uyvx, uyvy, uyvz, uzvx, uzvy, uzvz;

    /* init */
    vec3x_init(&p);
    vec3x_init(&q);
    vec3x_init(&u);
    vec3x_init(&v);
    mpz_init(pxqx);
    mpz_init(pxqy);
    mpz_init(pxqz);
    mpz_init(pyqx);
    mpz_init(pyqy);
    mpz_init(pyqz);
    mpz_init(pzqx);
    mpz_init(pzqy);
    mpz_init(pzqz);
    mpz_init(uxvx);
    mpz_init(uxvy);
    mpz_init(uxvz);
    mpz_init(uyvx);
    mpz_init(uyvy);
    mpz_init(uyvz);
    mpz_init(uzvx);
    mpz_init(uzvy);
    mpz_init(uzvz);

    /* p, q, u, v */
    predg3x_pquv(&p, &q, &u, &v, pg);

    /* multiplies */
    mpz_mul(pxqx, p.x, q.x);
    mpz_mul(pxqy, p.x, q.y);
    mpz_mul(pxqz, p.x, q.z);
    mpz_mul(pyqx, p.y, q.x);
    mpz_mul(pyqy, p.y, q.y);
    mpz_mul(pyqz, p.y, q.z);
    mpz_mul(pzqx, p.z, q.x);
    mpz_mul(pzqy, p.z, q.y);
    mpz_mul(pzqz, p.z, q.z);
    mpz_mul(uxvx, u.x, v.x);
    mpz_mul(uxvy, u.x, v.y);
    mpz_mul(uxvz, u.x, v.z);
    mpz_mul(uyvx, u.y, v.x);
    mpz_mul(uyvy, u.y, v.y);
    mpz_mul(uyvz, u.y, v.z);
    mpz_mul(uzvx, u.z, v.x);
    mpz_mul(uzvy, u.z, v.y);
    mpz_mul(uzvz, u.z, v.z);

    /* base spin quadric */
    /* s12^2 */
    mpz_add(sq->a11, pzqz, uzvz);
    mpz_sub(sq->a11, sq->a11, pxqx);
    mpz_sub(sq->a11, sq->a11, pyqy);
    mpz_sub(sq->a11, sq->a11, uxvx);
    mpz_sub(sq->a11, sq->a11, uyvy);

    /* s23^2 */
    mpz_add(sq->a22, pxqx, uxvx);
    mpz_sub(sq->a22, sq->a22, pyqy);
    mpz_sub(sq->a22, sq->a22, pzqz);
    mpz_sub(sq->a22, sq->a22, uyvy);
    mpz_sub(sq->a22, sq->a22, uzvz);

    /* s31^2 */
    mpz_add(sq->a33, pyqy, uyvy);
    mpz_sub(sq->a33, sq->a33, pzqz);
    mpz_sub(sq->a33, sq->a33, pxqx);
    mpz_sub(sq->a33, sq->a33, uzvz);
    mpz_sub(sq->a33, sq->a33, uxvx);

    /* s0^2 */
    mpz_add(sq->a44, pxqx, uxvx);
    mpz_add(sq->a44, sq->a44, pyqy);
    mpz_add(sq->a44, sq->a44, pzqz);
    mpz_add(sq->a44, sq->a44, uyvy);
    mpz_add(sq->a44, sq->a44, uzvz);

    /* s23 s12 */
    mpz_add(sq->a12, pzqx, uzvx);
    mpz_add(sq->a12, sq->a12, pxqz);
    mpz_add(sq->a12, sq->a12, uxvz);

    /* s31 s12 */
    mpz_add(sq->a13, pyqz, uyvz);
    mpz_add(sq->a13, sq->a13, pzqy);
    mpz_add(sq->a13, sq->a13, uzvy);

    /* s0 s12 */
    mpz_add(sq->a14, pxqy, uxvy);
    mpz_sub(sq->a14, sq->a14, pyqx);
    mpz_sub(sq->a14, sq->a14, uyvx);

    /* s23 s31 */
    mpz_add(sq->a23, pxqy, uxvy);
    mpz_add(sq->a23, sq->a23, pyqx);
    mpz_add(sq->a23, sq->a23, uyvx);

    /* s23 s0 */
    mpz_add(sq->a24, pyqz, uyvz);
    mpz_sub(sq->a24, sq->a24, pzqy);
    mpz_sub(sq->a24, sq->a24, uzvy);

    /* s0 s31 */
    mpz_add(sq->a34, pzqx, uzvx);
    mpz_sub(sq->a34, sq->a34, pxqz);
    mpz_sub(sq->a34, sq->a34, uxvz);

    /* reduced spin-quadric */
    mpz_add(sq->a11, sq->a11, pg->c);
    mpz_add(sq->a22, sq->a22, pg->c);
    mpz_add(sq->a33, sq->a33, pg->c);
    mpz_add(sq->a44, sq->a44, pg->c);

    /* clear */
    vec3x_clear(&p);
    vec3x_clear(&q);
    vec3x_clear(&u);
    vec3x_clear(&v);
    mpz_clear(pxqx);
    mpz_clear(pxqy);
    mpz_clear(pxqz);
    mpz_clear(pyqx);
    mpz_clear(pyqy);
    mpz_clear(pyqz);
    mpz_clear(pzqx);
    mpz_clear(pzqy);
    mpz_clear(pzqz);
    mpz_clear(uxvx);
    mpz_clear(uxvy);
    mpz_clear(uxvz);
    mpz_clear(uyvx);
    mpz_clear(uyvy);
    mpz_clear(uyvz);
    mpz_clear(uzvx);
    mpz_clear(uzvy);
    mpz_clear(uzvz);
}

void spinquad3x_eval(mpz_ptr v, const struct spinquad3x_s *sq, const struct pin3x_s *p)
{
    mpz_t t;
    mpz_init(t);
    mpz_mul(v, p->p12, p->p23);
    mpz_mul(v, v, sq->a12);
    mpz_mul(t, p->p12, p->p31);
    mpz_mul(t, t, sq->a13);
    mpz_add(v, v, t);
    mpz_mul(t, p->p12, p->p0);
    mpz_mul(t, t, sq->a14);
    mpz_add(v, v, t);
    mpz_mul(t, p->p23, p->p31);
    mpz_mul(t, t, sq->a23);
    mpz_add(v, v, t);
    mpz_mul(t, p->p23, p->p0);
    mpz_mul(t, t, sq->a24);
    mpz_add(v, v, t);
    mpz_mul(t, p->p31, p->p0);
    mpz_mul(t, t, sq->a34);
    mpz_add(v, v, t);
    mpz_mul_2exp(v, v, 1);
    mpz_mul(t, p->p12, p->p12);
    mpz_mul(t, t, sq->a11);
    mpz_add(v, v, t);
    mpz_mul(t, p->p23, p->p23);
    mpz_mul(t, t, sq->a22);
    mpz_add(v, v, t);
    mpz_mul(t, p->p31, p->p31);
    mpz_mul(t, t, sq->a33);
    mpz_add(v, v, t);
    mpz_mul(t, p->p0, p->p0);
    mpz_mul(t, t, sq->a44);
    mpz_add(v, v, t);
    mpz_clear(t);
}
