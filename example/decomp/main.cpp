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
#include "../../plugin/decomp/decomp3f.h"
#include "cs2/plugin.h"
#include "cs2/timer.h"
#include <cstdio>
#include <cstdlib>

void load_mesh(struct decompmesh3f_s *m, const char *fp)
{
    int vs, fs, i, j;
    FILE *f = fopen(fp, "r");

    if (!f)
        return;

    if (fscanf(f, "%*s%d%d%*d",&vs,&fs) != 2)
        return;

    m->v = (struct cs2_vec3f_s *)malloc(sizeof(struct cs2_vec3f_s) * vs);
    m->vs = vs;

    for (i = 0; i < vs; ++i)
        if (fscanf(f, "%lf%lf%lf", &m->v[i].x, &m->v[i].y, &m->v[i].z) != 3)
            return;

    m->f = (struct decompface3f_s *)malloc(sizeof(struct decompface3f_s) * fs);
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
    // decomp
    cs2_plugin_ldpath(".");

    void *pl = cs2_plugin_load("libdecomp.so");

    if (!pl)
    {
        printf("missing plugin\n");
        return 1;
    }

    decomp3f_init_f pl_init = (decomp3f_init_f)cs2_plugin_func(pl, DECOMP3F_INIT_F_SYM);
    decomp3f_make_f pl_make = (decomp3f_make_f)cs2_plugin_func(pl, DECOMP3F_MAKE_F_SYM);
    decomp3f_clear_f pl_clear = (decomp3f_clear_f)cs2_plugin_func(pl, DECOMP3F_CLEAR_F_SYM);

    struct decomp3f_s d;

    struct decompmesh3f_s dm;
    load_mesh(&dm, "../data/mushroom.off");

    pl_init(&d);

    uint64_t start = cs2_timer_usec();

    pl_make(&d, &dm);

    uint64_t end = cs2_timer_usec();

    printf("decomposition took %lu usecs; sub-meshes: %d\n", static_cast<unsigned long>(end - start), static_cast<int>(d.ms));

    pl_clear(&d);

    cs2_plugin_unload(pl);

    free(dm.f);
    free(dm.v);

    return 0;
}
