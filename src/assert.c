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
#include <ctype.h>
#include <unistd.h>
#include <execinfo.h>
#include <dlfcn.h>
#include <link.h>

static const char *_cs2_color_default = CS2_COLOR_DEFAULT;
static const char *_cs2_color_lib = CS2_COLOR_LIGHT_BLUE;
static const char *_cs2_color_assert = CS2_COLOR_LIGHT_PURPLE;
static const char *_cs2_color_panic = CS2_COLOR_LIGHT_RED;
static const char *_cs2_color_warn = CS2_COLOR_LIGHT_YELLOW;

#if defined(__linux__)

size_t convert_to_vma(size_t addr)
{
  Dl_info info;
  struct link_map* link_map;

  dladdr1((void*)addr, &info, (void**)&link_map, RTLD_DL_LINKMAP);

  return addr-link_map->l_addr;
}

#endif /* defined(__linux__) */

#if defined(__FreeBSD__) || defined(__OpenBSD__) || defined(__NetBSD__) || defined(__DragonFly__)

size_t convert_to_vma(size_t addr)
{
  return addr;
}

#endif /* defined(__FreeBSD__) || defined(__OpenBSD__) || defined(__NetBSD__) || defined(__DragonFly__) */

void dump_stacktrace(int omit) {
    void* frames[128];

    int i, count = backtrace(frames, sizeof(frames) / sizeof(frames[0]));
    char **symbols = backtrace_symbols(frames, count);

    fprintf(stderr, "%sstacktrace%s:\n", _cs2_color_lib, _cs2_color_default);

    for (i = omit; i < count; ++i)
    {
        Dl_info info;

        fprintf(stderr, "[%d] %s", i, symbols[i]);

        if (dladdr(frames[i], &info))
        {
            char command[256];
            FILE *pipe;
            char ch;

            size_t vma_addr = convert_to_vma((size_t)frames[i]);
            vma_addr -= 1;

            snprintf(command, sizeof(command), "addr2line -e %s -Ci %zx", info.dli_fname, vma_addr);
            pipe = popen(command, "r");

            if (pipe) {
                fprintf(stderr, " @ ");

                while ((ch = fgetc(pipe)) != EOF)
                    if (isprint(ch))
                        fputc(ch, stderr);

                pclose(pipe);
            }
        }

        fputc('\n', stderr);
    }

    free(symbols);
}

void cs2_assert(int value, const char *cond, const char *file, int line)
{
    if (value)
        return;

    fprintf(stderr, "%slibcs2:%s %sassertion '%s' failed at %s:%d%s\n",
           _cs2_color_lib, _cs2_color_default, _cs2_color_assert, cond, file, line, _cs2_color_default);

    dump_stacktrace(1);

    fflush(stderr);

    abort();
}

void cs2_assert_msg(int value, const char *cond, const char *file, int line, const char *msg, ...)
{
    va_list args;

    if (value)
        return;

    fprintf(stderr, "%slibcs2:%s %sassertion '%s' failed at %s:%d with message '",
           _cs2_color_lib, _cs2_color_default, _cs2_color_assert, cond, file, line);

    va_start(args, msg);
    vfprintf(stderr, msg, args);
    va_end(args);

    fprintf(stderr, "%s'\n", _cs2_color_default);

    dump_stacktrace(1);

    fflush(stderr);

    abort();
}

void cs2_panic_msg(const char *file, int line, const char *msg, ...)
{
    va_list args;

    fprintf(stderr, "%slibcs2:%s %spanic at %s:%d with message '",
           _cs2_color_lib, _cs2_color_default, _cs2_color_panic, file, line);

    va_start(args, msg);
    vfprintf(stderr, msg, args);
    va_end(args);

    fprintf(stderr, "%s'\n", _cs2_color_default);

    dump_stacktrace(1);

    fflush(stderr);

    abort();
}

void cs2_warn_msg(const char *file, int line, const char *msg, ...)
{
    va_list args;

    fprintf(stderr, "%slibcs2:%s %swarning at %s:%d with message '",
           _cs2_color_lib, _cs2_color_default, _cs2_color_warn, file, line);

    va_start(args, msg);
    vfprintf(stderr, msg, args);
    va_end(args);

    fprintf(stderr, "%s'\n", _cs2_color_default);

    dump_stacktrace(1);

    fflush(stderr);
}
