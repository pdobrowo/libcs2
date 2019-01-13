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
#include "cs2/preds3f.h"

void cs2_preds3f_set(struct cs2_preds3f_s *s, const struct cs2_vec3f_s *k, const struct cs2_vec3f_s *l, const struct cs2_vec3f_s *a, const struct cs2_vec3f_s *b)
{
    cs2_vec3f_copy(&s->k, k);
    cs2_vec3f_copy(&s->l, l);
    cs2_vec3f_copy(&s->a, a);
    cs2_vec3f_copy(&s->b, b);
}

void cs2_preds3f_copy(struct cs2_preds3f_s *r, const struct cs2_preds3f_s *s)
{
    cs2_vec3f_copy(&r->k, &s->k);
    cs2_vec3f_copy(&r->l, &s->l);
    cs2_vec3f_copy(&r->a, &s->a);
    cs2_vec3f_copy(&r->b, &s->b);
}
