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
#ifndef LIBCS2_PLUGIN_DECOMP3F_H
#define LIBCS2_PLUGIN_DECOMP3F_H

#include "cs2/vec3f.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

struct decompmesh3f_s
{
    /* vertices */
    vec3f_t *v;
    size_t vs;

    /* indices */
    size_t *i;
    size_t is;
};

typedef struct decompmesh3f_s decompmesh3f_t;

enum decomptype3f_e
{
    decomptype3f_success,

    decomptype3f_undefined,
    decomptype3f_internal_error,
    decomptype3f_out_of_memory
};

typedef enum decomptype3f_e decomptype3f_t;

struct decomp3f_s
{
    decomptype3f_t t;

    /* meshes */
    decompmesh3f_t *m;
    size_t ms;
};

typedef struct decomp3f_s decomp3f_t;

__attribute__((visibility("default"))) void decomp3f_init(decomp3f_t *d);
__attribute__((visibility("default"))) void decomp3f(decomp3f_t *d, const decompmesh3f_t *dm);
__attribute__((visibility("default"))) void decomp3f_clear(decomp3f_t *d);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBCS2_PLUGIN_DECOMP3F_H */
