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
project(cs2)
cmake_minimum_required(VERSION 2.8.12)

include(CheckCCompilerFlag)

# modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# deps: gmp (system)
find_package(GMP REQUIRED)
include_directories(${GMP_INCLUDE_DIR})

# deps: qhull (local)
set(QHULL_BASE_DIR ${CMAKE_SOURCE_DIR}/deps/qhull)
include_directories(${QHULL_BASE_DIR}/include)

# compiler flags
set(COMPILER_FLAGS "-Wall -Wextra -Wno-long-long -Wformat=2")

# standard
add_definitions(-D_GNU_SOURCE)

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
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

# internal
include_directories(${CMAKE_SOURCE_DIR}/inc)

# inc
set(cs2_HEADERS
    # floating-point arithmetic
    inc/cs2/vec3f.h
    inc/cs2/vec4f.h
    inc/cs2/mat33f.h
    inc/cs2/mat44f.h
    inc/cs2/plane3f.h
    inc/cs2/plane4f.h
    inc/cs2/pin3f.h
    inc/cs2/spin3f.h
    inc/cs2/spinquad3f.h
    inc/cs2/predh3f.h
    inc/cs2/preds3f.h
    inc/cs2/predg3f.h
    inc/cs2/predbb3f.h
    inc/cs2/predtt3f.h
    inc/cs2/predcc3f.h
    inc/cs2/predmm3f.h
    inc/cs2/bezierqq1f.h
    inc/cs2/bezierqq4f.h
    inc/cs2/beziertreeqq4f.h
    inc/cs2/hull4f.h
    inc/cs2/mathf.h

    # exact integer arithmetic
    inc/cs2/vec3x.h
    inc/cs2/plane3x.h
    inc/cs2/pin3x.h
    inc/cs2/spinquad3x.h
    inc/cs2/predh3x.h
    inc/cs2/preds3x.h
    inc/cs2/predg3x.h

    # other
    inc/cs2/defs.h
    inc/cs2/plugin.h
    inc/cs2/timer.h
    inc/cs2/rand.h
    inc/cs2/mem.h
    inc/cs2/fmt.h
    inc/cs2/assert.h
    inc/cs2/color.h
)

# src
set(cs2_SOURCES
    ${cs2_HEADERS}

    # floating-point arithmetic
    src/vec3f.c
    src/vec4f.c
    src/mat33f.c
    src/mat44f.c
    src/plane3f.c
    src/plane4f.c
    src/pin3f.c
    src/spin3f.c
    src/spinquad3f.c
    src/predh3f.c
    src/preds3f.c
    src/predg3f.c
    src/predbb3f.c
    src/predtt3f.c
    src/predcc3f.c
    src/predmm3f.c
    src/bezierqq1f.c
    src/bezierqq4f.c
    src/beziertreeqq4f.c
    src/hull4f.c
    src/mathf.c

    # exact integer arithmetic
    src/vec3x.c
    src/plane3x.c
    src/pin3x.c
    src/spinquad3x.c
    src/predh3x.c
    src/preds3x.c
    src/predg3x.c

    # other
    src/plugin.c
    src/timer.c
    src/rand.c
    src/mem.c
    src/fmt.c
    src/assert.c
)

add_library(cs2 SHARED ${cs2_SOURCES})
add_library(cs2_s STATIC ${cs2_SOURCES})

set_target_properties(cs2 PROPERTIES C_VISIBILITY_PRESET hidden)
target_link_libraries(cs2 c m)

# deps: gmp (system)
target_link_libraries(cs2 ${GMP_LIBRARIES})

# deps: qhull (local)
target_link_libraries(cs2 ${QHULL_BASE_DIR}/lib/libqhullstatic_r.a)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_link_libraries(cs2 dl)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

if(${CMAKE_SYSTEM_NAME} MATCHES "FreeBSD" OR
   ${CMAKE_SYSTEM_NAME} MATCHES "OpenBSD" OR
   ${CMAKE_SYSTEM_NAME} MATCHES "NetBSD" OR
   ${CMAKE_SYSTEM_NAME} MATCHES "DragonFly")
    # link: backtrace, backtrace_symbols
    target_link_libraries(cs2 execinfo)
endif(${CMAKE_SYSTEM_NAME} MATCHES "FreeBSD" OR
      ${CMAKE_SYSTEM_NAME} MATCHES "OpenBSD" OR
      ${CMAKE_SYSTEM_NAME} MATCHES "NetBSD" OR
      ${CMAKE_SYSTEM_NAME} MATCHES "DragonFly")

message("CS2 system: ${CMAKE_SYSTEM_NAME}")
message("CS2 system version: ${CMAKE_SYSTEM_VERSION}")
message("CS2 compiler: ${CMAKE_C_COMPILER_ID}")
message("CS2 processor: ${CMAKE_SYSTEM_PROCESSOR}")
