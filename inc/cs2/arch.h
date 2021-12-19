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
#ifndef CS2_ASSERT_H
#define CS2_ASSERT_H

/* Linux */
#if defined(__linux__)
#  define CS2_ARCH_LINUX
#endif /* defined(__linux__)*/

/* FreeBSD */
#if defined(__FreeBSD__)
#  define CS2_ARCH_FREEBSD
#  define CS2_ARCH_BSD
#endif /* defined(__FreeBSD__) */

/* OpenBSD */
#if defined(__OpenBSD__)
#  define CS2_ARCH_OPENBSD
#  define CS2_ARCH_BSD
#endif /* defined(__OpenBSD__) */

/* OpenBSD */
#if defined(__NetBSD__)
#  define CS2_ARCH_NETBSD
#  define CS2_ARCH_BSD
#endif /* defined(__NetBSD__) */

/* DragonFly */
#if defined(__DragonFly__)
#  define CS2_ARCH_DRAGONFLY
#  define CS2_ARCH_BSD
#endif /* defined(__DragonFly__) */

/* Solaris + SunOS */
#if defined(sun) || defined(__sun)
#  if defined(__SVR4) || defined(__svr4__)
#    define CS2_ARCH_SOLARIS
#  else /* defined(__SVR4) || defined(__svr4__) */
#    define CS2_ARCH_SUNOS
#  endif /* defined(__SVR4) || defined(__svr4__) */
#endif /* defined(sun) || defined(__sun) */

#endif /* CS2_ASSERT_H */
