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
#include "cs2/vec4f.h"
#include "cs2/fmt.h"
#include "cs2/assert.h"
#include <math.h>

void cs2_vec4f_set(struct cs2_vec4f_s *v, double x, double y, double z, double w)
{
    v->x = x;
    v->y = y;
    v->z = z;
    v->w = w;
}

void cs2_vec4f_zero(struct cs2_vec4f_s *v)
{
    cs2_vec4f_set(v, 0, 0, 0, 0);
}

double cs2_vec4f_coord(const struct cs2_vec4f_s *v, int i)
{
    switch (i)
    {
    case 0: return v->x;
    case 1: return v->y;
    case 2: return v->z;
    case 3: return v->w;
    }
    CS2_PANIC_MSG("invalid coordinate");
    return 0.0;
}

void cs2_vec4f_copy(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va)
{
    v->x = va->x;
    v->y = va->y;
    v->z = va->z;
    v->w = va->w;
}

void cs2_vec4f_add(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va, const struct cs2_vec4f_s *vb)
{
    v->x = va->x + vb->x;
    v->y = va->y + vb->y;
    v->z = va->z + vb->z;
    v->w = va->w + vb->w;
}

void cs2_vec4f_sub(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va, const struct cs2_vec4f_s *vb)
{
    v->x = va->x - vb->x;
    v->y = va->y - vb->y;
    v->z = va->z - vb->z;
    v->w = va->w - vb->w;
}

void cs2_vec4f_neg(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va)
{
    v->x = -va->x;
    v->y = -va->y;
    v->z = -va->z;
    v->w = -va->w;
}

void cs2_vec4f_mul(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va, double sa)
{
    v->x = va->x * sa;
    v->y = va->y * sa;
    v->z = va->z * sa;
    v->w = va->w * sa;
}

void cs2_vec4f_mad2(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va, double sa, const struct cs2_vec4f_s *vb, double sb)
{
    v->x = va->x * sa + vb->x * sb;
    v->y = va->y * sa + vb->y * sb;
    v->z = va->z * sa + vb->z * sb;
    v->w = va->w * sa + vb->w * sb;
}

void cs2_vec4f_mad3(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va, double sa, const struct cs2_vec4f_s *vb, double sb, const struct cs2_vec4f_s *vc, double sc)
{
    v->x = va->x * sa + vb->x * sb + vc->x * sc;
    v->y = va->y * sa + vb->y * sb + vc->y * sc;
    v->z = va->z * sa + vb->z * sb + vc->z * sc;
    v->w = va->w * sa + vb->w * sb + vc->w * sc;
}

void cs2_vec4f_mad4(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va, double sa, const struct cs2_vec4f_s *vb, double sb, const struct cs2_vec4f_s *vc, double sc, const struct cs2_vec4f_s *vd, double sd)
{
    v->x = va->x * sa + vb->x * sb + vc->x * sc + vd->x * sd;
    v->y = va->y * sa + vb->y * sb + vc->y * sc + vd->y * sd;
    v->z = va->z * sa + vb->z * sb + vc->z * sc + vd->z * sd;
    v->w = va->w * sa + vb->w * sb + vc->w * sc + vd->w * sd;
}

void cs2_vec4f_mad5(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va, double sa, const struct cs2_vec4f_s *vb, double sb, const struct cs2_vec4f_s *vc, double sc, const struct cs2_vec4f_s *vd, double sd, const struct cs2_vec4f_s *ve, double se)
{
    v->x = va->x * sa + vb->x * sb + vc->x * sc + vd->x * sd + ve->x * se;
    v->y = va->y * sa + vb->y * sb + vc->y * sc + vd->y * sd + ve->y * se;
    v->z = va->z * sa + vb->z * sb + vc->z * sc + vd->z * sd + ve->z * se;
    v->w = va->w * sa + vb->w * sb + vc->w * sc + vd->w * sd + ve->w * se;
}

double cs2_vec4f_dot(const struct cs2_vec4f_s *va, const struct cs2_vec4f_s *vb)
{
    return va->x * vb->x + va->y * vb->y + va->z * vb->z + va->w * vb->w;
}

void cs2_vec4f_unit(struct cs2_vec4f_s *v, const struct cs2_vec4f_s *va)
{
    double len = cs2_vec4f_len(va);
    CS2_ASSERT_MSG(len > 0.0, "vector must be non-zero");
    cs2_vec4f_mul(v, va, 1.0 / len);
}

double cs2_vec4f_len(const struct cs2_vec4f_s *v)
{
    return sqrt(v->x * v->x + v->y * v->y + v->z * v->z + v->w * v->w);
}

double cs2_vec4f_sqlen(const struct cs2_vec4f_s *v)
{
    return v->x * v->x + v->y * v->y + v->z * v->z + v->w * v->w;
}

double cs2_vec4f_tr(const struct cs2_vec4f_s *v)
{
    return v->x + v->y + v->z + v->w;
}

void cs2_vec4f_from_pin3f(struct cs2_vec4f_s *v, const struct cs2_pin3f_s *p)
{
    v->x = p->p12;
    v->y = p->p23;
    v->z = p->p31;
    v->w = p->p0;
}

void cs2_vec4f_print_json(const struct cs2_vec4f_s *v, FILE *f, size_t indent)
{
    cs2_fmt_indent(indent, f);
    fprintf(f, "{ \"x\": %.2f, \"y\": %.2f, \"z\": %.2f, \"w\": %.2f }", v->x, v->y, v->z, v->w);
}
