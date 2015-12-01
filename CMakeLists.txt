###
 # CS2 library
 #
 # Copyright 2015 Przemyslaw Dobrowolski <przemyslaw.dobrowolski@xtb.com>
 #
 ##
project(cs2)
cmake_minimum_required(VERSION 2.6)

# modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# apr and apr-util
find_package(GMP REQUIRED)
include_directories(${GMP_INCLUDE_DIR})

# compiler settings
add_definitions(
    -Wall
    -Wextra
    -pedantic
    -ansi
    -Wno-long-long
)

# optimizations
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fomit-frame-pointer -march=native")

# output directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

# internal
include_directories(${CMAKE_SOURCE_DIR}/inc)

# inc
set(cs2_HEADERS
    # floating-point arithmetic
    inc/cs2/vec3f.h
    inc/cs2/vec4f.h
    inc/cs2/plane3f.h
    inc/cs2/spin3f.h
    inc/cs2/spinquad3f.h
    inc/cs2/predh3f.h
    inc/cs2/preds3f.h
    inc/cs2/predg3f.h

    # exact integer arithmetic
    inc/cs2/vec3x.h
)

# src
set(cs2_SOURCES
    ${cs2_HEADERS}

    # floating-point arithmetic
    src/vec3f.c
    src/vec4f.c
    src/plane3f.c
    src/spin3f.c
    src/spinquad3f.c
    src/predh3f.c
    src/preds3f.c
    src/predg3f.c

    # exact integer arithmetic
    src/vec3x.c
)

add_library(cs2 STATIC ${cs2_SOURCES})

message("CS2 system: ${CMAKE_SYSTEM_NAME}")
message("CS2 system version: ${CMAKE_SYSTEM_VERSION}")
message("CS2 compiler: ${CMAKE_CXX_COMPILER_ID}")
message("CS2 processor: ${CMAKE_SYSTEM_PROCESSOR}")
