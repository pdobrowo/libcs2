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
#include "cs2/assert.h"
#include <stdio.h>
#include <stdlib.h>

void cs2_assert(int value, const char *cond, const char *file, int line)
{
    if (value)
        return;

    printf("libcs2: assertion '%s' failed at %s:%d\n", cond, file, line);
    fflush(stdout);

    abort();
}

void cs2_assert_msg(int value, const char *cond, const char *msg, const char *file, int line)
{
    if (value)
        return;

    printf("libcs2: assertion '%s' failed at %s:%d with message '%s'\n", cond, file, line, msg);
    fflush(stdout);

    abort();
}

void cs2_assert_panic(const char *msg, const char *file, int line)
{
    printf("libcs2: panic at %s:%d with message '%s'\n", file, line, msg);
    fflush(stdout);

    abort();
}
