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
#include "cs2/pin3f.h"
#include <math.h>

const struct cs2_pin3f_s CS2_PIN3F_ZERO = { 0.0, 0.0, 0.0, 0.0 };
const struct cs2_pin3f_s CS2_PIN3F_ONE = { 0.0, 0.0, 0.0, 1.0 };

void cs2_pin3f_set(struct cs2_pin3f_s *p, double p12, double p23, double p31, double p0)
{
    p->p12 = p12;
    p->p23 = p23;
    p->p31 = p31;
    p->p0 = p0;
}

void cs2_pin3f_zero(struct cs2_pin3f_s *p)
{
    cs2_pin3f_set(p, 0, 0, 0, 0);
}

void cs2_pin3f_add(struct cs2_pin3f_s *p, const struct cs2_pin3f_s *pa, const struct cs2_pin3f_s *pb)
{
    p->p12 = pa->p12 + pb->p12;
    p->p23 = pa->p23 + pb->p23;
    p->p31 = pa->p31 + pb->p31;
    p->p0 = pa->p0 + pb->p0;
}

void cs2_pin3f_sub(struct cs2_pin3f_s *p, const struct cs2_pin3f_s *pa, const struct cs2_pin3f_s *pb)
{
    p->p12 = pa->p12 - pb->p12;
    p->p23 = pa->p23 - pb->p23;
    p->p31 = pa->p31 - pb->p31;
    p->p0 = pa->p0 - pb->p0;
}

void cs2_pin3f_neg(struct cs2_pin3f_s *p, const struct cs2_pin3f_s *pa)
{
    p->p12 = -pa->p12;
    p->p23 = -pa->p23;
    p->p31 = -pa->p31;
    p->p0 = -pa->p0;
}

void cs2_pin3f_mul(struct cs2_pin3f_s *p, const struct cs2_pin3f_s *pa, double sa)
{
    p->p12 = pa->p12 * sa;
    p->p23 = pa->p23 * sa;
    p->p31 = pa->p31 * sa;
    p->p0 = pa->p0 * sa;
}

void cs2_pin3f_cl(struct cs2_pin3f_s *p, const struct cs2_vec3f_s *va, const struct cs2_vec3f_s *vb)
{
    p->p12 = va->x * vb->y - va->y * vb->x;
    p->p23 = va->y * vb->z - va->z * vb->y;
    p->p31 = va->z * vb->x - va->x * vb->z;
    p->p0 = va->x * vb->x + va->y * vb->y + va->z * vb->z;
}

void cs2_pin3f_mad2(struct cs2_pin3f_s *p, const struct cs2_pin3f_s *pa, double sa, const struct cs2_pin3f_s *pb, double sb)
{
    p->p12 = pa->p12 * sa + pb->p12 * sb;
    p->p23 = pa->p23 * sa + pb->p23 * sb;
    p->p31 = pa->p31 * sa + pb->p31 * sb;
    p->p0 = pa->p0 * sa + pb->p0 * sb;
}

void cs2_pin3f_mad3(struct cs2_pin3f_s *p, const struct cs2_pin3f_s *pa, double sa, const struct cs2_pin3f_s *vb, double sb, const struct cs2_pin3f_s *pc, double sc)
{
    p->p12 = pa->p12 * sa + vb->p12 * sb + pc->p12 * sc;
    p->p23 = pa->p23 * sa + vb->p23 * sb + pc->p23 * sc;
    p->p31 = pa->p31 * sa + vb->p31 * sb + pc->p31 * sc;
    p->p0 = pa->p0 * sa + vb->p0 * sb + pc->p0 * sc;
}

void cs2_pin3f_mad4(struct cs2_pin3f_s *p, const struct cs2_pin3f_s *pa, double sa, const struct cs2_pin3f_s *pb, double sb, const struct cs2_pin3f_s *pc, double sc, const struct cs2_pin3f_s *pd, double sd)
{
    p->p12 = pa->p12 * sa + pb->p12 * sb + pc->p12 * sc + pd->p12 * sd;
    p->p23 = pa->p23 * sa + pb->p23 * sb + pc->p23 * sc + pd->p23 * sd;
    p->p31 = pa->p31 * sa + pb->p31 * sb + pc->p31 * sc + pd->p31 * sd;
    p->p0 = pa->p0 * sa + pb->p0 * sb + pc->p0 * sc + pd->p0 * sd;
}

void cs2_pin3f_mad5(struct cs2_pin3f_s *p, const struct cs2_pin3f_s *pa, double sa, const struct cs2_pin3f_s *pb, double sb, const struct cs2_pin3f_s *pc, double sc, const struct cs2_pin3f_s *pd, double sd, const struct cs2_pin3f_s *pe, double se)
{
    p->p12 = pa->p12 * sa + pb->p12 * sb + pc->p12 * sc + pd->p12 * sd + pe->p12 * se;
    p->p23 = pa->p23 * sa + pb->p23 * sb + pc->p23 * sc + pd->p23 * sd + pe->p23 * se;
    p->p31 = pa->p31 * sa + pb->p31 * sb + pc->p31 * sc + pd->p31 * sd + pe->p31 * se;
    p->p0 = pa->p0 * sa + pb->p0 * sb + pc->p0 * sc + pd->p0 * sd + pe->p0 * se;
}

double cs2_pin3f_len(const struct cs2_pin3f_s *p)
{
    return sqrt(p->p12 * p->p12 + p->p23 * p->p23 + p->p31 * p->p31 + p->p0 * p->p0);
}

double cs2_pin3f_sqlen(const struct cs2_pin3f_s *p)
{
    return p->p12 * p->p12 + p->p23 * p->p23 + p->p31 * p->p31 + p->p0 * p->p0;
}
