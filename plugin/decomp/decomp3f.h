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

#include "cs2/defs.h"
#include "cs2/vec3f.h"
#include <stddef.h>

CS2_API_BEGIN

struct decompface3f_s
{
    /* indices */
    size_t *i;
    size_t is;
};

struct decompmesh3f_s
{
    /* vertices */
    struct vec3f_s *v;
    size_t vs;

    /* faces */
    struct decompface3f_s *f;
    size_t fs;
};

enum decomptype3f_e
{
    decomptype3f_success,

    decomptype3f_undefined,
    decomptype3f_internal_error,
    decomptype3f_out_of_memory
};

struct decomp3f_s
{
    enum decomptype3f_e t;

    /* meshes */
    struct decompmesh3f_s *m;
    size_t ms;
};

CS2_API void decomp3f_init(struct decomp3f_s *d);
CS2_API void decomp3f_make(struct decomp3f_s *d, const struct decompmesh3f_s *dm);
CS2_API void decomp3f_clear(struct decomp3f_s *d);

typedef void (*decomp3f_init_f)(struct decomp3f_s *d);
typedef void (*decomp3f_make_f)(struct decomp3f_s *d, const struct decompmesh3f_s *dm);
typedef void (*decomp3f_clear_f)(struct decomp3f_s *d);

#define DECOMP3F_INIT_F_SYM "decomp3f_init"
#define DECOMP3F_MAKE_F_SYM "decomp3f_make"
#define DECOMP3F_CLEAR_F_SYM "decomp3f_clear"

CS2_API_END

#endif /* LIBCS2_PLUGIN_DECOMP3F_H */
