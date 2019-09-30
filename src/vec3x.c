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
#include "cs2/vec3x.h"
#include "cs2/assert.h"

void cs2_vec3x_init(struct cs2_vec3x_s *v)
{
    mpz_init(v->x);
    mpz_init(v->y);
    mpz_init(v->z);
}

void cs2_vec3x_clear(struct cs2_vec3x_s *v)
{
    mpz_clear(v->x);
    mpz_clear(v->y);
    mpz_clear(v->z);
}

void cs2_vec3x_set(struct cs2_vec3x_s *v, mpz_srcptr x, mpz_srcptr y, mpz_srcptr z)
{
    mpz_set(v->x, x);
    mpz_set(v->y, y);
    mpz_set(v->z, z);
}

void cs2_vec3x_set_si(struct cs2_vec3x_s *v, long x, long y, long z)
{
    mpz_set_si(v->x, x);
    mpz_set_si(v->y, y);
    mpz_set_si(v->z, z);
}

void cs2_vec3x_zero(struct cs2_vec3x_s *v)
{
    mpz_set_si(v->x, 0);
    mpz_set_si(v->y, 0);
    mpz_set_si(v->z, 0);
}

int cs2_vec3x_is_zero(const struct cs2_vec3x_s *v)
{
    return !mpz_sgn(v->x) && !mpz_sgn(v->y) && !mpz_sgn(v->z);
}

void cs2_vec3x_copy(struct cs2_vec3x_s *v, const struct cs2_vec3x_s *va)
{
    mpz_set(v->x, va->x);
    mpz_set(v->y, va->y);
    mpz_set(v->z, va->z);
}

void cs2_vec3x_add(struct cs2_vec3x_s *v, const struct cs2_vec3x_s *va, const struct cs2_vec3x_s *vb)
{
    mpz_add(v->x, va->x, vb->x);
    mpz_add(v->y, va->y, vb->y);
    mpz_add(v->z, va->z, vb->z);
}

void cs2_vec3x_sub(struct cs2_vec3x_s *v, const struct cs2_vec3x_s *va, const struct cs2_vec3x_s *vb)
{
    mpz_sub(v->x, va->x, vb->x);
    mpz_sub(v->y, va->y, vb->y);
    mpz_sub(v->z, va->z, vb->z);
}

void cs2_vec3x_neg(struct cs2_vec3x_s *v, const struct cs2_vec3x_s *va)
{
    mpz_neg(v->x, va->x);
    mpz_neg(v->y, va->y);
    mpz_neg(v->z, va->z);
}

void cs2_vec3x_mul(struct cs2_vec3x_s *v, const struct cs2_vec3x_s *va, mpz_srcptr sa)
{
    mpz_mul(v->x, va->x, sa);
    mpz_mul(v->y, va->y, sa);
    mpz_mul(v->z, va->z, sa);
}

void cs2_vec3x_cl(struct cs2_pin3x_s *v, const struct cs2_vec3x_s *va, const struct cs2_vec3x_s *vb)
{
    mpz_t t;
    mpz_init(t);
    mpz_mul(v->p12, va->x, vb->y);
    mpz_mul(t, va->y, vb->x);
    mpz_sub(v->p12, v->p12, t);
    mpz_mul(v->p23, va->y, vb->z);
    mpz_mul(t, va->z, vb->y);
    mpz_sub(v->p23, v->p23, t);
    mpz_mul(v->p31, va->z, vb->x);
    mpz_mul(t, va->x, vb->z);
    mpz_sub(v->p31, v->p31, t);
    mpz_mul(v->p0, va->x, vb->x);
    mpz_mul(t, va->y, vb->y);
    mpz_add(v->p0, v->p0, t);
    mpz_mul(t, va->z, vb->z);
    mpz_add(v->p0, v->p0, t);
    mpz_clear(t);
}

void cs2_vec3x_mad2(struct cs2_vec3x_s *v, const struct cs2_vec3x_s *va, mpz_srcptr sa, const struct cs2_vec3x_s *vb, mpz_srcptr sb)
{
    mpz_t t;
    mpz_init(t);
    mpz_mul(v->x, va->x, sa);
    mpz_mul(t, vb->x, sb);
    mpz_add(v->x, v->x, t);
    mpz_mul(v->y, va->y, sa);
    mpz_mul(t, vb->y, sb);
    mpz_add(v->y, v->y, t);
    mpz_mul(v->z, va->z, sa);
    mpz_mul(t, vb->z, sb);
    mpz_add(v->z, v->z, t);
    mpz_clear(t);
}

void cs2_vec3x_mad3(struct cs2_vec3x_s *v, const struct cs2_vec3x_s *va, mpz_srcptr sa, const struct cs2_vec3x_s *vb, mpz_srcptr sb, const struct cs2_vec3x_s *vc, mpz_srcptr sc)
{
    mpz_t t;
    mpz_init(t);
    mpz_mul(v->x, va->x, sa);
    mpz_mul(t, vb->x, sb);
    mpz_add(v->x, v->x, t);
    mpz_mul(t, vc->x, sc);
    mpz_add(v->x, v->x, t);
    mpz_mul(v->y, va->y, sa);
    mpz_mul(t, vb->y, sb);
    mpz_add(v->y, v->y, t);
    mpz_mul(t, vc->y, sc);
    mpz_add(v->y, v->y, t);
    mpz_mul(v->z, va->z, sa);
    mpz_mul(t, vb->z, sb);
    mpz_add(v->z, v->z, t);
    mpz_mul(t, vc->z, sc);
    mpz_add(v->z, v->z, t);
    mpz_clear(t);
}

void cs2_vec3x_mad4(struct cs2_vec3x_s *v, const struct cs2_vec3x_s *va, mpz_srcptr sa, const struct cs2_vec3x_s *vb, mpz_srcptr sb, const struct cs2_vec3x_s *vc, mpz_srcptr sc, const struct cs2_vec3x_s *vd, mpz_srcptr sd)
{
    mpz_t t;
    mpz_init(t);
    mpz_mul(v->x, va->x, sa);
    mpz_mul(t, vb->x, sb);
    mpz_add(v->x, v->x, t);
    mpz_mul(t, vc->x, sc);
    mpz_add(v->x, v->x, t);
    mpz_mul(t, vd->x, sd);
    mpz_add(v->x, v->x, t);
    mpz_mul(v->y, va->y, sa);
    mpz_mul(t, vb->y, sb);
    mpz_add(v->y, v->y, t);
    mpz_mul(t, vc->y, sc);
    mpz_add(v->y, v->y, t);
    mpz_mul(t, vd->y, sd);
    mpz_add(v->y, v->y, t);
    mpz_mul(v->z, va->z, sa);
    mpz_mul(t, vb->z, sb);
    mpz_add(v->z, v->z, t);
    mpz_mul(t, vc->z, sc);
    mpz_add(v->z, v->z, t);
    mpz_mul(t, vd->z, sd);
    mpz_add(v->z, v->z, t);
    mpz_clear(t);
}

void cs2_vec3x_mad5(struct cs2_vec3x_s *v, const struct cs2_vec3x_s *va, mpz_srcptr sa, const struct cs2_vec3x_s *vb, mpz_srcptr sb, const struct cs2_vec3x_s *vc, mpz_srcptr sc, const struct cs2_vec3x_s *vd, mpz_srcptr sd, const struct cs2_vec3x_s *ve, mpz_srcptr se)
{
    mpz_t t;
    mpz_init(t);
    mpz_mul(v->x, va->x, sa);
    mpz_mul(t, vb->x, sb);
    mpz_add(v->x, v->x, t);
    mpz_mul(t, vc->x, sc);
    mpz_add(v->x, v->x, t);
    mpz_mul(t, vd->x, sd);
    mpz_add(v->x, v->x, t);
    mpz_mul(t, ve->x, se);
    mpz_add(v->x, v->x, t);
    mpz_mul(v->y, va->y, sa);
    mpz_mul(t, vb->y, sb);
    mpz_add(v->y, v->y, t);
    mpz_mul(t, vc->y, sc);
    mpz_add(v->y, v->y, t);
    mpz_mul(t, vd->y, sd);
    mpz_add(v->y, v->y, t);
    mpz_mul(t, ve->y, se);
    mpz_add(v->y, v->y, t);
    mpz_mul(v->z, va->z, sa);
    mpz_mul(t, vb->z, sb);
    mpz_add(v->z, v->z, t);
    mpz_mul(t, vc->z, sc);
    mpz_add(v->z, v->z, t);
    mpz_mul(t, vd->z, sd);
    mpz_add(v->z, v->z, t);
    mpz_mul(t, ve->z, se);
    mpz_add(v->z, v->z, t);
    mpz_clear(t);
}

void cs2_vec3x_dot(mpz_ptr s, const struct cs2_vec3x_s *va, const struct cs2_vec3x_s *vb)
{
    mpz_t t;
    mpz_init(t);
    mpz_mul(s, va->x, vb->x);
    mpz_mul(t, va->y, vb->y);
    mpz_add(s, s, t);
    mpz_mul(t, va->z, vb->z);
    mpz_add(s, s, t);
    mpz_clear(t);
}

void cs2_vec3x_cross(struct cs2_vec3x_s *v, const struct cs2_vec3x_s *va, const struct cs2_vec3x_s *vb)
{
    CS2_ASSERT(va != vb);
    mpz_t t;
    mpz_init(t);
    mpz_mul(v->x, va->y, vb->z);
    mpz_mul(t, va->z, vb->y);
    mpz_sub(v->x, v->x, t);
    mpz_mul(v->y, va->z, vb->x);
    mpz_mul(t, va->x, vb->z);
    mpz_sub(v->y, v->y, t);
    mpz_mul(v->z, va->x, vb->y);
    mpz_mul(t, va->y, vb->x);
    mpz_sub(v->z, v->z, t);
    mpz_clear(t);
}

void cs2_vec3x_sqlen(mpz_ptr s, const struct cs2_vec3x_s *v)
{
    mpz_t t;
    mpz_init(t);
    mpz_mul(s, v->x, v->x);
    mpz_mul(t, v->y, v->y);
    mpz_add(s, s, t);
    mpz_mul(t, v->z, v->z);
    mpz_add(s, s, t);
    mpz_clear(t);
}

void cs2_vec3x_tr(mpz_ptr s, const struct cs2_vec3x_s *v)
{
    mpz_add(s, v->x, v->y);
    mpz_add(s, s, v->z);
}
