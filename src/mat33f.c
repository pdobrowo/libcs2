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
#include "cs2/mat33f.h"
#include "cs2/assert.h"
#include <stddef.h>

void cs2_mat33f_zero(struct cs2_mat33f_s *m)
{
    m->e00 = m->e01 = m->e02 = 0.0;
    m->e10 = m->e11 = m->e12 = 0.0;
    m->e20 = m->e21 = m->e22 = 0.0;
}

void cs2_mat33f_identity(struct cs2_mat33f_s *m)
{
    m->e00 = m->e11 = m->e22 = 1.0;
    m->e01 = m->e02 = 0.0;
    m->e10 = m->e12 = 0.0;
    m->e20 = m->e21 = 0.0;
}

void cs2_mat33f_transform(struct cs2_vec3f_s *v, const struct cs2_mat33f_s *ma, const struct cs2_vec3f_s *va)
{
    CS2_ASSERT(v != va);
    v->x = ma->e00 * va->x + ma->e01 * va->y + ma->e02 * va->z;
    v->y = ma->e10 * va->x + ma->e11 * va->y + ma->e12 * va->z;
    v->z = ma->e20 * va->x + ma->e21 * va->y + ma->e22 * va->z;
}
