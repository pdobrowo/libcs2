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
#ifndef CS2_TEST_H
#define CS2_TEST_H

#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include "cmocka.h"

struct test_case_s
{
    const char *name;
    void (*proc)(void **);
    struct test_case_s *next;
};

struct test_suite_s
{
    const char *name;
    int (*init_proc)(void **);
    int (*clean_proc)(void **);
    struct test_case_s *test_cases;
    struct test_suite_s *next;
};

#define TEST_SUITE_REGISTER(TestSuiteName) \
    static void test_suite_reg_##TestSuiteName(void) __attribute__((constructor)); \
    static void test_suite_reg_##TestSuiteName(void) \
    { \
        extern struct test_suite_s *test_suites_registry; \
        if (!test_suites_registry) \
            test_suites_registry = &test_suite_##TestSuiteName; \
        else \
        { \
            test_suite_##TestSuiteName.next = test_suites_registry; \
            test_suites_registry = &test_suite_##TestSuiteName; \
        } \
    }

#define TEST_SUITE(TestSuiteName) \
    static struct test_suite_s test_suite_##TestSuiteName = { #TestSuiteName, NULL, NULL, NULL, NULL }; \
    TEST_SUITE_REGISTER(TestSuiteName)

#define TEST_SUITE_INIT_CLEAN(TestSuiteName, TestSuiteInit, TestSuiteClean) \
    static struct test_suite_s test_suite_##TestSuiteName = { #TestSuiteName, TestSuiteInit, TestSuiteClean, NULL, NULL }; \
    TEST_SUITE_REGISTER(TestSuiteName);

#define TEST_CASE(TestSuiteName, TestCaseName) \
    static void test_case_trampoline_##TestSuiteName##_##TestCaseName(void **state); \
    static void test_case_proc_##TestSuiteName##_##TestCaseName(void); \
    static struct test_case_s test_case_##TestSuiteName##_##TestCaseName = { #TestCaseName, &test_case_trampoline_##TestSuiteName##_##TestCaseName, NULL }; \
    static void test_case_reg_##TestSuiteName##_##TestCaseName(void) __attribute__((constructor)); \
    static void test_case_reg_##TestSuiteName##_##TestCaseName(void) \
    { \
        if (!test_suite_##TestSuiteName.test_cases) \
            test_suite_##TestSuiteName.test_cases = &test_case_##TestSuiteName##_##TestCaseName; \
        else \
        { \
            test_case_##TestSuiteName##_##TestCaseName.next = test_suite_##TestSuiteName.test_cases; \
            test_suite_##TestSuiteName.test_cases = &test_case_##TestSuiteName##_##TestCaseName; \
        } \
    } \
    static void test_case_trampoline_##TestSuiteName##_##TestCaseName(void **state) \
    { \
        (void)state; \
        test_case_proc_##TestSuiteName##_##TestCaseName(); \
    } \
    static void test_case_proc_##TestSuiteName##_##TestCaseName(void)

#define TEST_ASSERT_TRUE assert_true
#define TEST_ASSERT_STRING_EQUAL assert_string_equal

/* suites */
struct test_suite_s *test_suites(void);

#endif /* CS2_TEST_H */
