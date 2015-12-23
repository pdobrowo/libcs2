/**
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015-2016 Przemysław Dobrowolski
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
#include "cs2/vec3f.h"
#include "cs2/vec3x.h"
#include "cs2/spinquad3f.h"
#include "cs2/predh3f.h"
#include "../../plugin/decomp/decomp3f.h"
#include "cs2/plugin.h"
#include "cs2/timer.h"
#include <cstdio>
#include <cstdlib>

void load_mesh(decompmesh3f_t *m, const char *fp)
{
    int vs, fs, i, j;
    FILE *f = fopen(fp, "r");

    if (!f)
        return;

    if (fscanf(f, "%*s%d%d%*d",&vs,&fs) != 2)
        return;

    m->v = (vec3f_t *)malloc(sizeof(vec3f_t) * vs);
    m->vs = vs;

    for (i = 0; i < vs; ++i)
        if (fscanf(f, "%lf%lf%lf", &m->v[i].x, &m->v[i].y, &m->v[i].z) != 3)
            return;

    m->f = (decompface3f_t *)malloc(sizeof(decompface3f_t) * fs);
    m->fs = fs;

    for (i = 0; i < fs; ++i)
    {
        int is;

        if (fscanf(f, "%d", &is) != 1)
            return;

        m->f[i].i = (size_t *)malloc(sizeof(size_t) * is);
        m->f[i].is = is;

        for (j = 0; j < is; ++j)
        {
            double x;

            if (fscanf(f, "%lf", &x) != 1)
                return;

            m->f[i].i[j] = x;
        }
    }

    fclose(f);
}

int main()
{
    vec3f_t a = { 1, 2, 3 };
    vec3f_t b = { 4, 5, 6 };
    vec3f_t r;

    vec3x_t u, v, w;

    vec3f_add(&r, &a, &b);

double xxx = vec3f_tr(&r);

    vec3x_init(&u);
    vec3x_init(&v);
    vec3x_init(&w);

    vec3x_set_si(&u, 1, 2, 3);
    vec3x_set_si(&v, 4, 5, 6);

    vec3x_add(&w, &u, &v);

    vec3x_clear(&u);
    vec3x_clear(&v);
    vec3x_clear(&w);

    // test
    predh3f_t p;

    vec3f_set(&p.b, 1, 2, 3);
    vec3f_set(&p.p.n, -1, -1, 2);
    p.p.d = 1;

    spinquad3f_t s;
    spinquad3f_from_predh3f(&s, &p);

    predg3f_t g = { { 1, 2, 3 }, { -1, 0, 2 }, { 4, 2, -2 }, { 0, -2, 3 }, 1 };
    predgtype3f_e gt = predg3f_type(&g);
    (void)gt;

    spinquad3f_t gs;
    spinquad3f_from_predg3f(&gs, &g);

    mat44f_t em;
    vec4f_t ev;
    predg3f_eigen(&em, &ev, &g);

    predgparam3f_t par;
    predg3f_param(&par, &g);

    double sqnorm_min = 999;
    double sqnorm_max = -999;
    double val_min = 999;
    double val_max = -999;

    for (double pu = 0; pu < 1; pu += 0.01) for (double pv = 0; pv < 1; pv += 0.01)
    {
        spin3f_t sp;
        predgparam3f_eval(&sp, &par, pu, pv, 1);

        double sqnorm = sp.s12 * sp.s12 + sp.s23 * sp.s23 + sp.s31 * sp.s31 + sp.s0 * sp.s0;

        if (sqnorm < sqnorm_min)
            sqnorm_min = sqnorm;

        if (sqnorm > sqnorm_max)
            sqnorm_max = sqnorm;

        double val = spinquad3f_eval(&gs, &sp);

        if (val < val_min)
            val_min = val;

        if (val > val_max)
            val_max = val;
    }

    printf("sqnorm_min: %.20f\n", sqnorm_min);
    printf("sqnorm_max: %.20f\n", sqnorm_max);

    printf("val_min: %.20f\n", val_min);
    printf("val_max: %.20f\n", val_max);

    // decomp
    plugin_ldpath(".");

    void *pl = plugin_load("libdecomp.so");

    decomp3f_init_f pl_init = (decomp3f_init_f)plugin_fn(pl, DECOMP3F_INIT_F_SYM);
    decomp3f_make_f pl_make = (decomp3f_make_f)plugin_fn(pl, DECOMP3F_MAKE_F_SYM);
    decomp3f_clear_f pl_clear = (decomp3f_clear_f)plugin_fn(pl, DECOMP3F_CLEAR_F_SYM);

    decomp3f_t d;

    decompmesh3f_t dm;
    load_mesh(&dm, "../data/mushroom.off");

    pl_init(&d);

    uint64_t start = timer_usec();

    pl_make(&d, &dm);

    uint64_t end = timer_usec();

    printf("decomposition took %llu usecs; sub-meshes: %d\n", end - start, static_cast<int>(d.ms));

    pl_clear(&d);

    plugin_unload(pl);

    return 0;
}
