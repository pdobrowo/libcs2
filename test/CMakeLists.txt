##
 # Copyright (c) 2015-2019 Przemysław Dobrowolski
 #
 # This file is part of the Configuration Space Library (libcs2), a library
 # for creating configuration spaces of various motion planning problems.
 #
 # Permission is hereby granted, free of charge, to any person obtaining a copy
 # of this software and associated documentation files (the "Software"), to deal
 # in the Software without restriction, including without limitation the rights
 # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 # copies of the Software, and to permit persons to whom the Software is
 # furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included in
 # all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 # SOFTWARE.
 ##
project(test)
cmake_minimum_required(VERSION 2.6)

# modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/../cmake)

# includes
include(CheckCCompilerFlag)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    include(CodeCoverage)
endif()

# deps: cs2 (local)
find_package(CS2 REQUIRED)
include_directories(${CS2_INCLUDE_DIR})

# deps: cmocka (local)
set(CMOCKA_BASE_DIR ${CMAKE_SOURCE_DIR}/../deps/cmocka)
include_directories(${CMOCKA_BASE_DIR}/include)

# code coverage
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    append_coverage_compiler_flags()
endif()

# compiler flags
set(COMPILER_FLAGS "-Wall -Wextra -Wno-long-long -Wformat=2 -Wno-variadic-macros")

# standard
add_definitions(-D_GNU_SOURCE)

# standard
check_c_compiler_flag("-std=c99" COMPILER_SUPPORTS_C99)

if(COMPILER_SUPPORTS_C99)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")
else()
    message(STATUS "The compiler ${CMAKE_C_COMPILER} has no C99 support. Please use a different C compiler.")
endif()

# optimizations
check_c_compiler_flag(-Ofast COMPILER_SUPPORT_OFAST)

if(COMPILER_SUPPORT_OFAST)
    set(COMPILER_RELEASE_OPTIMIZATION -Ofast)
else(COMPILER_SUPPORT_OFAST)
    set(COMPILER_RELEASE_OPTIMIZATION -O3)
endif(COMPILER_SUPPORT_OFAST)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMPILER_FLAGS}")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${COMPILER_RELEASE_OPTIMIZATION} -fomit-frame-pointer -mtune=native")

# output directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/../bin)

# internal
include_directories(${CMAKE_SOURCE_DIR}/inc)

# inc
set(test_HEADERS
    # runner
    inc/test/test.h

    # test
    inc/test/testpredg3f.h
)

# src
set(test_SOURCES
    ${test_HEADERS}

    # runner
    src/main.c
    src/test.c

    # test
    src/testpredg3f.c

    # suites
    src/bezierqq1f.c
    src/beziertreeqq4f.c
    src/hull4f.c
    src/vec3f.c
    src/vec3x.c
    src/predg3f.c
    src/pin3f.c
)

add_executable(test ${test_SOURCES})

# deps: cs2 (local)
target_link_libraries(test ${CS2_STATIC_LIBRARIES})

# deps: cmocka (local)
target_link_libraries(test ${CMOCKA_BASE_DIR}/lib/libcmocka.a)

# code coverage
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    setup_target_for_coverage_gcovr_html(NAME test_coverage EXECUTABLE test DEPENDENCIES test)
endif()
