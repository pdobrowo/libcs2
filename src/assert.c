/**
 * Copyright (c) 2015-2019 Przemysław Dobrowolski
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
#include "cs2/color.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>

static const char *_cs2_color_default = CS2_COLOR_DEFAULT;
static const char *_cs2_color_lib = CS2_COLOR_LIGHT_BLUE;
static const char *_cs2_color_assert = CS2_COLOR_LIGHT_PURPLE;
static const char *_cs2_color_panic = CS2_COLOR_LIGHT_RED;
static const char *_cs2_color_warn = CS2_COLOR_LIGHT_YELLOW;

void cs2_assert(int value, const char *cond, const char *file, int line)
{
    if (value)
        return;

    printf("%slibcs2:%s %sassertion '%s' failed at %s:%d%s\n",
           _cs2_color_lib, _cs2_color_default, _cs2_color_assert, cond, file, line, _cs2_color_default);
    fflush(stdout);

    abort();
}

void cs2_assert_msg(int value, const char *cond, const char *file, int line, const char *msg, ...)
{
    va_list args;

    if (value)
        return;

    printf("%slibcs2:%s %sassertion '%s' failed at %s:%d with message '",
           _cs2_color_lib, _cs2_color_default, _cs2_color_assert, cond, file, line);

    va_start(args, msg);
    vprintf(msg, args);
    va_end(args);

    printf("%s'\n", _cs2_color_default);

    fflush(stdout);

    abort();
}

void cs2_panic_msg(const char *file, int line, const char *msg, ...)
{
    va_list args;

    printf("%slibcs2:%s %spanic at %s:%d with message '",
           _cs2_color_lib, _cs2_color_default, _cs2_color_panic, file, line);

    va_start(args, msg);
    vprintf(msg, args);
    va_end(args);

    printf("%s'\n", _cs2_color_default);

    fflush(stdout);

    abort();
}

void cs2_warn_msg(const char *file, int line, const char *msg, ...)
{
    va_list args;

    printf("%slibcs2:%s %swarning at %s:%d with message '",
           _cs2_color_lib, _cs2_color_default, _cs2_color_warn, file, line);

    va_start(args, msg);
    vprintf(msg, args);
    va_end(args);

    printf("%s'\n", _cs2_color_default);

    fflush(stdout);
}
