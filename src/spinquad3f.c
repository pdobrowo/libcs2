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
#include "cs2/spinquad3f.h"

void spinquad3f_from_predh3f(spinquad3f_t *s, const predh3f_t *p)
{
    predg3f_t g;
    predg3f_from_predh3f(&g, p);
    spinquad3f_from_predg3f(s, &g);
}

void spinquad3f_from_preds3f(spinquad3f_t *s, const preds3f_t *p)
{
    predg3f_t g;
    predg3f_from_preds3f(&g, p);
    spinquad3f_from_predg3f(s, &g);
}

void spinquad3f_from_predg3f(spinquad3f_t *s, const predg3f_t *p)
{
    double kx = p->k.x;
    double ky = p->k.y;
    double kz = p->k.z;

    double lx = p->l.x;
    double ly = p->l.y;
    double lz = p->l.z;

    double ax = p->a.x;
    double ay = p->a.y;
    double az = p->a.z;

    double bx = p->b.x;
    double by = p->b.y;
    double bz = p->b.z;

    /* p, q, u, v */
    double px = ky * lz - kz * ly;
    double py = kz * lx - kx * lz;
    double pz = kx * ly - ky * lx;

    double qx = ax - bx;
    double qy = ay - by;
    double qz = az - bz;

    double ux = kx - lx;
    double uy = ky - ly;
    double uz = kz - lz;

    double vx = ay * bz - az * by;
    double vy = az * bx - ax * bz;
    double vz = ax * by - ay * bx;

    /* multiplies */
    double pxqx = px * qx;
    double pxqy = px * qy;
    double pxqz = px * qz;
    double pyqx = py * qx;
    double pyqy = py * qy;
    double pyqz = py * qz;
    double pzqx = pz * qx;
    double pzqy = pz * qy;
    double pzqz = pz * qz;

    double uxvx = ux * vx;
    double uxvy = ux * vy;
    double uxvz = ux * vz;
    double uyvx = uy * vx;
    double uyvy = uy * vy;
    double uyvz = uy * vz;
    double uzvx = uz * vx;
    double uzvy = uz * vy;
    double uzvz = uz * vz;

    /* base spin quadric */
    s->a11 = - uxvx - uyvy + uzvz - pxqx - pyqy + pzqz; /* s12^2 */
    s->a22 =   uxvx - uyvy - uzvz + pxqx - pyqy - pzqz; /* s23^2 */
    s->a33 = - uxvx + uyvy - uzvz - pxqx + pyqy - pzqz; /* s31^2 */
    s->a44 =   uxvx + uyvy + uzvz + pxqx + pyqy + pzqz; /* s0^2 */
    s->a12 =   uxvz + pzqx + uzvx + pxqz; /* s23 s12 */
    s->a13 =   uyvz + pzqy + uzvy + pyqz; /* s31 s12 */
    s->a14 =   uxvy - uyvx - pyqx + pxqy; /* s0 s12 */
    s->a23 =   uxvy + uyvx + pyqx + pxqy; /* s23 s31 */
    s->a24 =   uyvz - pzqy - uzvy + pyqz; /* s23 s0 */
    s->a34 = - uxvz + pzqx + uzvx - pxqz; /* s0 s31 */

    /* reduced spin-quadric */
    s->a11 += p->c;
    s->a22 += p->c;
    s->a33 += p->c;
    s->a44 += p->c;
}
