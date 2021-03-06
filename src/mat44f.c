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
#include "cs2/mat44f.h"
#include "cs2/assert.h"
#include <stddef.h>

void cs2_mat44f_zero(struct cs2_mat44f_s *m)
{
    m->e00 = m->e01 = m->e02 = m->e03 = 0.0;
    m->e10 = m->e11 = m->e12 = m->e13 = 0.0;
    m->e20 = m->e21 = m->e22 = m->e23 = 0.0;
    m->e30 = m->e31 = m->e32 = m->e33 = 0.0;
}

void cs2_mat44f_identity(struct cs2_mat44f_s *m)
{
    m->e00 = m->e11 = m->e22 = m->e33 = 1.0;
    m->e01 = m->e02 = m->e03 = 0.0;
    m->e10 = m->e12 = m->e13 = 0.0;
    m->e20 = m->e21 = m->e23 = 0.0;
    m->e30 = m->e31 = m->e32 = 0.0;
}

void cs2_mat44f_transform(struct cs2_vec4f_s *v, const struct cs2_mat44f_s *ma, const struct cs2_vec4f_s *va)
{
    CS2_ASSERT(v != va);
    v->x = ma->e00 * va->x + ma->e01 * va->y + ma->e02 * va->z + ma->e03 * va->w;
    v->y = ma->e10 * va->x + ma->e11 * va->y + ma->e12 * va->z + ma->e13 * va->w;
    v->z = ma->e20 * va->x + ma->e21 * va->y + ma->e22 * va->z + ma->e23 * va->w;
    v->w = ma->e30 * va->x + ma->e31 * va->y + ma->e32 * va->z + ma->e33 * va->w;
}
