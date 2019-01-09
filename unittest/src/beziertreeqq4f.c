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
#include "cs2/beziertreeqq4f.h"
#include "cs2/predg3f.h"
#include "unittest/unittest.h"
#include <math.h>

#define EPS (10e-8)
#define test_almost_equal(x, y) TEST_ASSERT_TRUE(fabs((x) - (y)) < EPS)

struct predbb_func_s
{
    struct cs2_predg3f_s p;
    struct cs2_predgparam3f_s pp;
};

static void predbb_func(struct cs2_vec4f_s *r, double u, double v, void *data)
{
    struct predbb_func_s *f = (struct predbb_func_s *)data;
    struct cs2_spin3f_s s;
    cs2_predgparam3f_eval(&s, &f->pp, u, v, 0);
    r->x = s.s12;
    r->y = s.s23;
    r->z = s.s31;
    r->w = s.s0;
}

static void create_z_barrel(struct cs2_predg3f_s *p)
{
    /* an example z-barrel */
    cs2_vec3f_set(&p->k, 1.0, 1.0, 1.0);
    cs2_vec3f_set(&p->l, 1.0, 0.0, 2.0);
    cs2_vec3f_set(&p->a, 0.0, 1.0, 1.0);
    cs2_vec3f_set(&p->b, 0.0, 2.0, 1.0);
    p->c = 0.5;
}

TEST_SUITE(beziertreeqq4f)

TEST_CASE(beziertreeqq4f, init_and_sub)
{
    struct cs2_beziertreeqq4f_s t;
    struct predbb_func_s f;

    create_z_barrel(&f.p);

    cs2_predg3f_param(&f.pp, &f.p);

    /* init */
    cs2_beziertreeqq4f_init(&t);
    cs2_beziertreeqq4f_from_func(&t, &predbb_func, &f);

    /* initial tree is virtual with zero volume and zero area */
    TEST_ASSERT_TRUE(cs2_beziertreenodeqq4f_is_virt(t.rn));
    test_almost_equal(cs2_beziertreeqq4f_vol(&t), 0.0);
    test_almost_equal(cs2_beziertreeqq4f_area(&t), 0.0);

    /* do first level subdivision */
    cs2_beziertreenodeqq4f_sub(t.rn);

    /* first level subdivision must not be virtual and volume must not be zero */
    TEST_ASSERT_TRUE(!cs2_beziertreenodeqq4f_is_virt(t.rn->c[0][0]));
    TEST_ASSERT_TRUE(!cs2_beziertreenodeqq4f_is_virt(t.rn->c[0][1]));
    TEST_ASSERT_TRUE(!cs2_beziertreenodeqq4f_is_virt(t.rn->c[1][0]));
    TEST_ASSERT_TRUE(!cs2_beziertreenodeqq4f_is_virt(t.rn->c[1][1]));

    TEST_ASSERT_TRUE(cs2_beziertreeqq4f_vol(&t) > 0.0);
    TEST_ASSERT_TRUE(cs2_beziertreeqq4f_area(&t) > 0.0);

    /* do second level subdivision */
    cs2_beziertreenodeqq4f_sub(t.rn->c[0][0]);
    cs2_beziertreenodeqq4f_sub(t.rn->c[0][1]);
    cs2_beziertreenodeqq4f_sub(t.rn->c[1][0]);
    cs2_beziertreenodeqq4f_sub(t.rn->c[1][1]);

    /* clear */
    cs2_beziertreeqq4f_clear(&t);
}

TEST_CASE(beziertreeqq4f, sub_vol)
{
    struct cs2_beziertreeqq4f_s t;
    struct cs2_beziertreeleafsqq4f_s l;
    struct cs2_beziertreeleafqq4f_s *ll;
    struct predbb_func_s f;
    size_t pc, tc;

    create_z_barrel(&f.p);

    cs2_predg3f_param(&f.pp, &f.p);

    /* init */
    cs2_beziertreeqq4f_init(&t);
    cs2_beziertreeqq4f_from_func(&t, &predbb_func, &f);

    /* sub vol */
    cs2_beziertreeleafsqq4f_init(&l, &t);

    cs2_beziertreeleafsqq4f_sub_vol(&l, 0.0001);
    pc = l.c;

    cs2_beziertreeleafsqq4f_sub_vol(&l, 0.0001);

    TEST_ASSERT_TRUE(pc == l.c);

    /* count */
    tc = 0;
    ll = l.l;

    while (ll)
    {
        ++tc;
        ll = ll->next;
    }

    TEST_ASSERT_TRUE(pc == tc);

    /* clear */
    cs2_beziertreeqq4f_clear(&t);
}
