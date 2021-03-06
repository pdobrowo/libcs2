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
#include "cs2/spinquad3f.h"
#include "cs2/vec3f.h"
#include "cs2/assert.h"
#include <math.h>

void cs2_spinquad3f_from_predh3f(struct cs2_spinquad3f_s *sq, const struct cs2_predh3f_s *ph)
{
    struct cs2_predg3f_s g;
    cs2_predg3f_from_predh3f(&g, ph);
    cs2_spinquad3f_from_predg3f(sq, &g);
}

void cs2_spinquad3f_from_preds3f(struct cs2_spinquad3f_s *sq, const struct cs2_preds3f_s *ps)
{
    struct cs2_predg3f_s g;
    cs2_predg3f_from_preds3f(&g, ps);
    cs2_spinquad3f_from_predg3f(sq, &g);
}

void cs2_spinquad3f_from_predg3f(struct cs2_spinquad3f_s *sq, const struct cs2_predg3f_s *pg)
{
    struct cs2_vec3f_s p, q, u, v;
    double pxqx, pxqy, pxqz, pyqx, pyqy, pyqz, pzqx, pzqy, pzqz, uxvx, uxvy, uxvz, uyvx, uyvy, uyvz, uzvx, uzvy, uzvz;

    /* p, q, u, v */
    cs2_predg3f_pquv(&p, &q, &u, &v, pg);

    /* multiplies */
    pxqx = p.x * q.x;
    pxqy = p.x * q.y;
    pxqz = p.x * q.z;
    pyqx = p.y * q.x;
    pyqy = p.y * q.y;
    pyqz = p.y * q.z;
    pzqx = p.z * q.x;
    pzqy = p.z * q.y;
    pzqz = p.z * q.z;
    uxvx = u.x * v.x;
    uxvy = u.x * v.y;
    uxvz = u.x * v.z;
    uyvx = u.y * v.x;
    uyvy = u.y * v.y;
    uyvz = u.y * v.z;
    uzvx = u.z * v.x;
    uzvy = u.z * v.y;
    uzvz = u.z * v.z;

    /* base spin quadric */
    sq->a11 = - uxvx - uyvy + uzvz - pxqx - pyqy + pzqz; /* s12^2 */
    sq->a22 =   uxvx - uyvy - uzvz + pxqx - pyqy - pzqz; /* s23^2 */
    sq->a33 = - uxvx + uyvy - uzvz - pxqx + pyqy - pzqz; /* s31^2 */
    sq->a44 =   uxvx + uyvy + uzvz + pxqx + pyqy + pzqz; /* s0^2 */
    sq->a12 =   uxvz + pzqx + uzvx + pxqz; /* s23 s12 */
    sq->a13 =   uyvz + pzqy + uzvy + pyqz; /* s31 s12 */
    sq->a14 =   uxvy - uyvx - pyqx + pxqy; /* s0 s12 */
    sq->a23 =   uxvy + uyvx + pyqx + pxqy; /* s23 s31 */
    sq->a24 =   uyvz - pzqy - uzvy + pyqz; /* s23 s0 */
    sq->a34 = - uxvz + pzqx + uzvx - pxqz; /* s0 s31 */

    /* reduced spin-quadric */
    sq->a11 += pg->c;
    sq->a22 += pg->c;
    sq->a33 += pg->c;
    sq->a44 += pg->c;
}

double cs2_spinquad3f_eval(const struct cs2_spinquad3f_s *sq, const struct cs2_spin3f_s *s)
{
    return sq->a11 * s->s12 * s->s12 +
           sq->a22 * s->s23 * s->s23 +
           sq->a33 * s->s31 * s->s31 +
           sq->a44 * s->s0 * s->s0 +
           2 * (sq->a12 * s->s12 * s->s23 +
                sq->a13 * s->s12 * s->s31 +
                sq->a14 * s->s12 * s->s0 +
                sq->a23 * s->s23 * s->s31 +
                sq->a24 * s->s23 * s->s0 +
                sq->a34 * s->s31 * s->s0);
}

double cs2_spinquad3f_len(const struct cs2_spinquad3f_s *sq)
{
    return sqrt(sq->a11 * sq->a11 + sq->a12 * sq->a12 + sq->a13 * sq->a13 + sq->a14 * sq->a14 +
                sq->a22 * sq->a22 + sq->a23 * sq->a23 + sq->a24 * sq->a24 +
                sq->a33 * sq->a33 + sq->a34 * sq->a34 +
                sq->a44 * sq->a44);
}

double cs2_spinquad3f_sqlen(const struct cs2_spinquad3f_s *sq)
{
    return sq->a11 * sq->a11 + sq->a12 * sq->a12 + sq->a13 * sq->a13 + sq->a14 * sq->a14 +
           sq->a22 * sq->a22 + sq->a23 * sq->a23 + sq->a24 * sq->a24 +
           sq->a33 * sq->a33 + sq->a34 * sq->a34 +
           sq->a44 * sq->a44;
}

void cs2_spinquad3f_mul(struct cs2_spinquad3f_s *sq, const struct cs2_spinquad3f_s *sqa, double sa)
{
    sq->a11 = sqa->a11 * sa;
    sq->a12 = sqa->a12 * sa;
    sq->a13 = sqa->a13 * sa;
    sq->a14 = sqa->a14 * sa;
    sq->a22 = sqa->a22 * sa;
    sq->a23 = sqa->a23 * sa;
    sq->a24 = sqa->a24 * sa;
    sq->a33 = sqa->a33 * sa;
    sq->a34 = sqa->a34 * sa;
    sq->a44 = sqa->a44 * sa;
}

void cs2_spinquad3f_unit(struct cs2_spinquad3f_s *sq, const struct cs2_spinquad3f_s *sqa)
{
    double len = cs2_spinquad3f_len(sqa);
    CS2_ASSERT_MSG(len > 0.0, "vector must be non-zero");
    cs2_spinquad3f_mul(sq, sqa, 1.0 / len);
}
