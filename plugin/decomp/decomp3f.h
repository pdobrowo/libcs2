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

struct decompface3f_s
{
    /* indices */
    size_t *i;
    size_t is;
};

typedef struct decompface3f_s decompface3f_t;

struct decompmesh3f_s
{
    /* vertices */
    vec3f_t *v;
    size_t vs;

    /* faces */
    decompface3f_t *f;
    size_t fs;
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
__attribute__((visibility("default"))) void decomp3f_make(decomp3f_t *d, const decompmesh3f_t *dm);
__attribute__((visibility("default"))) void decomp3f_clear(decomp3f_t *d);

typedef void (*decomp3f_init_f)(decomp3f_t *d);
typedef void (*decomp3f_make_f)(decomp3f_t *d, const decompmesh3f_t *dm);
typedef void (*decomp3f_clear_f)(decomp3f_t *d);

#define DECOMP3F_INIT_F_SYM "decomp3f_init"
#define DECOMP3F_MAKE_F_SYM "decomp3f_make"
#define DECOMP3F_CLEAR_F_SYM "decomp3f_clear"

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBCS2_PLUGIN_DECOMP3F_H */