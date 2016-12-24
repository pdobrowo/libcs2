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
#ifndef CS2_MEM_H
#define CS2_MEM_H

#include "defs.h"
#include <stddef.h>

CS2_API_BEGIN

typedef void (*mem_error_func_t)(const char *file, int line, size_t size, const char *type);

CS2_API mem_error_func_t mem_at_error(mem_error_func_t f);
CS2_API void mem_default_error_func(const char *file, int line, size_t size, const char *type);
CS2_API void mem_trigger_error(const char *file, int line, size_t size, const char *type);

#define MEM_MALLOC(Type) \
    (__extension__( \
        { \
            void *ptr; \
            while (!(ptr = malloc(sizeof(Type)))) \
                mem_trigger_error(__FILE__, __LINE__, sizeof(Type), #Type); \
            (Type *)ptr; \
        } \
    ))

#define MEM_MALLOC_N(Type, N) \
    (__extension__( \
        { \
            void *ptr; \
            while (!(ptr = malloc(sizeof(Type) * N))) \
                mem_trigger_error(__FILE__, __LINE__, sizeof(Type), #Type); \
            (Type *)ptr; \
        } \
    ))

#define MEM_FREE(Ptr) \
    do { if (Ptr) free(Ptr); } while (0)

#endif /* CS2_MEM_H */
