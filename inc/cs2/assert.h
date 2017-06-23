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
#ifndef CS2_ASSERT_H
#define CS2_ASSERT_H

#include "defs.h"

CS2_API_BEGIN

CS2_API void cs2_assert(int value, const char *cond, const char *file, int line);
CS2_API void cs2_assert_msg(int value, const char *cond, const char *file, int line, const char *msg, ...);
CS2_API void cs2_assert_panic(const char *file, int line, const char *msg, ...);

#define CS2_ASSERT(Cond) do cs2_assert(Cond, #Cond, __FILE__, __LINE__); while (0)
#define CS2_ASSERT_MSG(Cond, Msg, ...) do cs2_assert_msg(Cond, #Cond, __FILE__, __LINE__, Msg, ## __VA_ARGS__); while (0)
#define CS2_ASSERT_PANIC(Msg, ...) do cs2_assert_panic(__FILE__, __LINE__, Msg, ## __VA_ARGS__); while (0)

CS2_API_END

#endif /* CS2_ASSERT_H */
