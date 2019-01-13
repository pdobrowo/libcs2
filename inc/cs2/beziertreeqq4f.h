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
#ifndef CS2_BEZIERTREEQQ4F_H
#define CS2_BEZIERTREEQQ4F_H

#include "defs.h"
#include "bezierqq4f.h"
#include "vec4f.h"

CS2_API_BEGIN

/**
 * spin bezier tree node
 */
struct cs2_beziertreenodeqq4f_s
{
    struct cs2_bezierqq4f_s b;

    double u0, u1, u2;
    double v0, v1, v2;

    double vol, area;

    struct cs2_beziertreeqq4f_s *r;
    struct cs2_beziertreenodeqq4f_s *p;

    /**
     * index (uv):
     *
     * 00 01
     * 10 11
     */
    struct cs2_beziertreenodeqq4f_s *c[2][2];
};

CS2_API void cs2_beziertreenodeqq4f_init(struct cs2_beziertreenodeqq4f_s *n, double u0, double u1, double u2, double v0, double v1, double v2,
                                     struct cs2_beziertreeqq4f_s *t, struct cs2_beziertreenodeqq4f_s *pn, int is_virt);
CS2_API void cs2_beziertreenodeqq4f_clear(struct cs2_beziertreenodeqq4f_s *n);

CS2_API void cs2_beziertreenodeqq4f_sub(struct cs2_beziertreenodeqq4f_s *n);
CS2_API int cs2_beziertreenodeqq4f_is_virt(struct cs2_beziertreenodeqq4f_s *n);
CS2_API int cs2_beziertreenodeqq4f_is_leaf(struct cs2_beziertreenodeqq4f_s *n);
CS2_API double cs2_beziertreenodeqq4f_vol(struct cs2_beziertreenodeqq4f_s *n);
CS2_API double cs2_beziertreenodeqq4f_area(struct cs2_beziertreenodeqq4f_s *n);

/**
 * spin bezier tree func
 */
typedef void (*cs2_beziertreeqq4f_func_t)(struct cs2_vec4f_s *r, double u, double v, void *d);

/**
 * spin bezier tree
 */
struct cs2_beziertreeqq4f_s
{
    cs2_beziertreeqq4f_func_t f;
    void *d;
    struct cs2_beziertreenodeqq4f_s *rn; /* virtual */
};

CS2_API void cs2_beziertreeqq4f_init(struct cs2_beziertreeqq4f_s *t);
CS2_API void cs2_beziertreeqq4f_clear(struct cs2_beziertreeqq4f_s *t);

CS2_API void cs2_beziertreeqq4f_from_func(struct cs2_beziertreeqq4f_s *t, cs2_beziertreeqq4f_func_t f, void *d);
CS2_API double cs2_beziertreeqq4f_vol(struct cs2_beziertreeqq4f_s *t);
CS2_API double cs2_beziertreeqq4f_area(struct cs2_beziertreeqq4f_s *t);

/**
 * spin bezier tree leaf
 */
struct cs2_beziertreeleafqq4f_s
{
    struct cs2_beziertreenodeqq4f_s *n;
    struct cs2_beziertreeleafqq4f_s *next;
};

struct cs2_beziertreeleafsqq4f_s
{
    struct cs2_beziertreeleafqq4f_s *l;
    size_t c;
};

CS2_API void cs2_beziertreeleafsqq4f_init(struct cs2_beziertreeleafsqq4f_s *l, struct cs2_beziertreeqq4f_s *t);
CS2_API void cs2_beziertreeleafsqq4f_clear(struct cs2_beziertreeleafsqq4f_s *l);

CS2_API void cs2_beziertreeleafsqq4f_sub_vol(struct cs2_beziertreeleafsqq4f_s *l, double vol);

CS2_API_END

#endif /* CS2_BEZIERTREEQQ4F_H */
