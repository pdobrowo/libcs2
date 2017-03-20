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
#include "cs2/mem.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static cs2_mem_error_func_t g_mem_error_func = &cs2_mem_default_error_func;

cs2_mem_error_func_t cs2_mem_at_error(cs2_mem_error_func_t f)
{
    cs2_mem_error_func_t pf = g_mem_error_func;
    g_mem_error_func = f;
    return pf;
}

void cs2_mem_default_error_func(const char *file, int line, size_t size, const char *type)
{
    const char *name = strrchr(file, '/');

    if (!name)
        name = file;
    else
        ++name;

    printf("libcs2: mem error at %s:%d while allocating %lu bytes for type %s\n", name, line, (unsigned long)size, type);
    fflush(stdout);

    /* retry after a second */
    usleep(1 * 1000 * 1000);
}

void cs2_mem_trigger_error(const char *file, int line, size_t size, const char *type)
{
    g_mem_error_func(file, line, size, type);
}
