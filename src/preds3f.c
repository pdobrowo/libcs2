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

void cs2_preds3f_set(struct cs2_preds3f_s *ps, const struct cs2_vec3f_s *vk, const struct cs2_vec3f_s *vl, const struct cs2_vec3f_s *va, const struct cs2_vec3f_s *vb)
{
    cs2_vec3f_copy(&ps->k, vk);
    cs2_vec3f_copy(&ps->l, vl);
    cs2_vec3f_copy(&ps->a, va);
    cs2_vec3f_copy(&ps->b, vb);
}

void cs2_preds3f_copy(struct cs2_preds3f_s *ps, const struct cs2_preds3f_s *psa)
{
    cs2_vec3f_copy(&ps->k, &psa->k);
    cs2_vec3f_copy(&ps->l, &psa->l);
    cs2_vec3f_copy(&ps->a, &psa->a);
    cs2_vec3f_copy(&ps->b, &psa->b);
}
