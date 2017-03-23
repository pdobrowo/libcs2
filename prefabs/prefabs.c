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
#include "prefabs.h"

void cs2_prefabs_predg3f_a_z_barrel_1(struct cs2_predg3f_s *p)
{
    /* predg3f_z_barrel_1 */
    cs2_vec3f_set(&p->k, -0.50,  0.00,  0.50);
    cs2_vec3f_set(&p->l,  0.33, -0.33,  0.17);
    cs2_vec3f_set(&p->a, -0.12,  0.08,  0.00);
    cs2_vec3f_set(&p->b,  0.40, -0.07, -0.71);
    p->c = 0.17;
}

void cs2_prefabs_predg3f_a_pair_of_separate_yz_caps_1(struct cs2_predg3f_s *p)
{
    /* predg3f_a_pair_of_separate_yz_caps */
    cs2_vec3f_set(&p->k, -0.50,  0.00,  0.50);
    cs2_vec3f_set(&p->l,  0.33, -0.33,  0.17);
    cs2_vec3f_set(&p->a,  0.54, -0.31,  0.00);
    cs2_vec3f_set(&p->b,  0.40, -0.60,  0.24);
    p->c = 0.17;
}
