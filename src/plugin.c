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
#include "cs2/plugin.h"
#include "cs2/mem.h"
#include "cs2/assert.h"
#include <string.h>
#include <stdlib.h>

#if defined(__linux__) || defined(__FreeBSD__) || defined(__OpenBSD__)

#include <dlfcn.h>

void *_cs2_plugin_load(const char *filename)
{
    return dlopen(filename, RTLD_NOW);
}

void _cs2_plugin_unload(void *handle)
{
    if (dlclose(handle))
        CS2_PANIC_MSG("dlclose() failed");
}

void *_cs2_plugin_sym(void *handle, const char *symbol)
{
	return dlsym(handle, symbol);
}

char *_cs2_plugin_realpath(const char *path)
{
    return realpath(path, 0);
}

#endif /* defined(__linux__) || defined(__FreeBSD__) || defined(__OpenBSD__) */

#if defined(__MINGW32__)

#define WIN32_LEAN_AND_MEAN
#define VC_EXTRALEAN

#include <windows.h>

void *_cs2_plugin_load(const char *filename)
{
    return (void *)LoadLibrary(filename);
}

void _cs2_plugin_unload(void *handle)
{
    if (!FreeLibrary(handle))
        CS2_PANIC_MSG("FreeLibrary() failed");
}

void *_cs2_plugin_sym(void *handle, const char *symbol)
{
    return (void *)GetProcAddress((HMODULE)handle, symbol);
}

char *_cs2_plugin_realpath(const char *path)
{
    char *canon = CS2_MEM_MALLOC_N(char, MAX_PATH + 1);

    DWORD rc = GetFullPathNameA(path, MAX_PATH, canon, NULL);

    if (!rc)
    {
        CS2_MEM_FREE(canon);
        return 0;
    }

    return canon;
}

#endif /* defined(__MINGW32__) */

#define PLUGIN_MAXPATH 1024

static char g_ldpath[PLUGIN_MAXPATH] = "";
static size_t g_ldpathlen = 0;

int cs2_plugin_ldpath(const char *p)
{
    size_t l = strlen(p);

    if (l >= sizeof(g_ldpath))
        return -1;

    memcpy(g_ldpath, p, l + 1);
    g_ldpathlen = l;

    return 0;
}

void *cs2_plugin_load(const char *f)
{
    size_t pl, fl;
    char *p, *r;
    void *d;

    /* real ld path */
    p = _cs2_plugin_realpath(g_ldpath);

    if (!p)
        return 0;

    pl = strlen(p);

    /* file name */
    fl = strlen(f);

    /* full path */
    r = (char *)CS2_MEM_MALLOC_N(char, pl + fl + 2);

    if (!r)
    {
        free(p);
        return 0;
    }

    memcpy(r, p, pl);
    r[pl] = '/';
    memcpy(r + pl + 1, f, fl + 1);

    d = _cs2_plugin_load(r);

    free(r);
    free(p);

    return d;
}

void *cs2_plugin_sym(void *p, const char *s)
{
    return _cs2_plugin_sym(p, s);
}

void cs2_plugin_unload(void *p)
{
    _cs2_plugin_unload(p);
}

cs2_plugin_func_t cs2_plugin_func(void *p, const char *s)
{
    cs2_plugin_func_t fn;
    void *sym = cs2_plugin_sym(p, s);
    CS2_ASSERT(sizeof(fn) == sizeof(sym));
    memcpy(&fn, &sym, sizeof(fn));
    return fn;
}
