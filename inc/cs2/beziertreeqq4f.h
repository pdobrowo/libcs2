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
#ifndef CS2_BEZIERTREEQQ4F_H
#define CS2_BEZIERTREEQQ4F_H

#include "defs.h"
#include "bezierqq4f.h"
#include "vec4f.h"

CS2_API_BEGIN

/**
 * spin bezier tree node
 */
struct beziertreenodeqq4f_s
{
    struct bezierqq4f_s b;

    double u0, u1, u2;
    double v0, v1, v2;

    double vol, area;

    struct beziertreeqq4f_s *r;
    struct beziertreenodeqq4f_s *p;

    /**
     * index (uv):
     *
     * 00 01
     * 10 11
     */
    struct beziertreenodeqq4f_s *c[2][2];
};

CS2_API void beziertreenodeqq4f_init(struct beziertreenodeqq4f_s *btn, double u0, double u1, double u2, double v0, double v1, double v2,
                                     struct beziertreeqq4f_s *r, struct beziertreenodeqq4f_s *p, int is_virt);
CS2_API void beziertreenodeqq4f_clear(struct beziertreenodeqq4f_s *btn);

CS2_API void beziertreenodeqq4f_sub(struct beziertreenodeqq4f_s *btn);
CS2_API int beziertreenodeqq4f_is_virt(struct beziertreenodeqq4f_s *btn);
CS2_API int beziertreenodeqq4f_is_leaf(struct beziertreenodeqq4f_s *btn);
CS2_API double beziertreenodeqq4f_vol(struct beziertreenodeqq4f_s *btn);
CS2_API double beziertreenodeqq4f_area(struct beziertreenodeqq4f_s *btn);

typedef void (*beziertreeqq4f_func_t)(struct vec4f_s *r, double u, double v, void *data);

struct beziertreeqq4f_s
{
    beziertreeqq4f_func_t f;
    void *data;
    struct beziertreenodeqq4f_s *r; /* virtual */
};

CS2_API void beziertreeqq4f_init(struct beziertreeqq4f_s *bt);
CS2_API void beziertreeqq4f_clear(struct beziertreeqq4f_s *bt);

CS2_API void beziertreeqq4f_from_func(struct beziertreeqq4f_s *bt, beziertreeqq4f_func_t f, void *data);
CS2_API double beziertreeqq4f_vol(struct beziertreeqq4f_s *btn);
CS2_API double beziertreeqq4f_area(struct beziertreeqq4f_s *btn);

CS2_API_END

#endif /* CS2_BEZIERTREEQQ4F_H */
