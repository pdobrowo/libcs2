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
#include "unittest/unittest.h"
#include "cs2/mem.h"
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "cmocka.h"

static size_t test_suite_count_test_cases(struct test_suite_s *test_suite)
{
    struct test_case_s *test_case = test_suite->test_cases;
    size_t count = 0;

    while (test_case)
    {
        ++count;
        test_case = test_case->next;
    }

    return count;
}

static size_t cm_tests_count()
{
    struct test_suite_s *suite = test_suites();
    size_t count = 0;

    while (suite)
    {
        count += test_suite_count_test_cases(suite);
        suite = suite->next;
    }

    return count;
}

static struct CMUnitTest *test_suite_fill_tests_cases(struct test_suite_s *test_suite, struct CMUnitTest *cm_tests)
{
    struct test_case_s *test_case = test_suite->test_cases;

    while (test_case)
    {
        char *name = MEM_MALLOC_N(char, strlen(test_suite->name) + 2 + strlen(test_case->name) + 1);

        strcpy(name, test_suite->name);
        strcat(name, "::");
        strcat(name, test_case->name);

        cm_tests->name = name;
        cm_tests->test_func = test_case->proc;

        cm_tests->setup_func = test_suite->init_proc;
        cm_tests->teardown_func = test_suite->clean_proc;

        cm_tests->initial_state = NULL;

        ++cm_tests;
        test_case = test_case->next;
    }

    return cm_tests;
}

static struct CMUnitTest *cm_tests_alloc()
{
    size_t count = cm_tests_count();
    struct CMUnitTest *cm_tests = MEM_MALLOC_N(struct CMUnitTest, count);
    struct CMUnitTest *cm_test = cm_tests;
    struct test_suite_s *suite = test_suites();

    while (suite)
    {
        cm_test = test_suite_fill_tests_cases(suite, cm_test);
        suite = suite->next;
    }

    return cm_tests;
}

static void cm_tests_free(struct CMUnitTest *cm_tests)
{
    size_t count = cm_tests_count();
    size_t i;

    for (i = 0; i < count; ++i)
        MEM_FREE((char *)cm_tests[i].name);

    MEM_FREE(cm_tests);
}

int invalid_usage(const char *msg)
{
    printf("error: %s'n", msg);
    printf("usage is:\n");
    printf("unittest [output={--stdout, --subunit, --tap, --xml}]\n");
    return EXIT_FAILURE;
}

int main(int argc, char *argv[])
{
    struct CMUnitTest *cm_tests;
    int ret;

    if (argc != 1 && argc != 2)
        return invalid_usage("expected no more than one parameter");

    if (argc == 2)
    {
        enum cm_message_output output;

        if (!strcmp(argv[1], "--stdout"))
            output = CM_OUTPUT_STDOUT;
        else if (!strcmp(argv[1], "--subunit"))
            output = CM_OUTPUT_SUBUNIT;
        else if (!strcmp(argv[1], "--tap"))
            output = CM_OUTPUT_TAP;
        else if (!strcmp(argv[1], "--xml"))
            output = CM_OUTPUT_XML;
        else
            return invalid_usage("invalid parameter value");

        cmocka_set_message_output(output);
    }

    cm_tests = cm_tests_alloc();
    ret = _cmocka_run_group_tests("unittest", cm_tests, cm_tests_count(), NULL, NULL);
    cm_tests_free(cm_tests);

    return ret;
}
