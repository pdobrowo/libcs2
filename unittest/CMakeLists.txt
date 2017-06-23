###
 # CS2 library
 #
 # Copyright 2015 Przemyslaw Dobrowolski <przemyslaw.dobrowolski@xtb.com>
 #
 ##
project(unittest)
cmake_minimum_required(VERSION 2.6)

include(CheckCCompilerFlag)

# modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/../cmake)

# deps: cs2 (local)
find_package(CS2 REQUIRED)
include_directories(${CS2_INCLUDE_DIR})

# deps: cmocka (local)
set(CMOCKA_BASE_DIR ${CMAKE_SOURCE_DIR}/../deps/cmocka)
include_directories(${CMOCKA_BASE_DIR}/include)

# compiler settings
add_definitions(
    -Wall
    -Wextra
    -pedantic
    -Wno-long-long
    -Wno-variadic-macros
)

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

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${COMPILER_RELEASE_OPTIMIZATION} -fomit-frame-pointer -march=native")

# output directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/../bin)

# internal
include_directories(${CMAKE_SOURCE_DIR}/inc)

# inc
set(unittest_HEADERS
    # runner
    inc/unittest/unittest.h
)

# src
set(unittest_SOURCES
    ${unittest_HEADERS}

    # runner
    src/main.c
    src/unittest.c

    # suites
    src/bezierqq1f.c
    src/beziertreeqq4f.c
    src/hull4f.c
    src/vec3f.c
    src/vec3x.c
    src/predg3f.c
)

add_executable(unittest ${unittest_SOURCES})

# deps: cs2 (local)
target_link_libraries(unittest ${CS2_STATIC_LIBRARIES})

# deps: cmocka (local)
target_link_libraries(unittest ${CMOCKA_BASE_DIR}/lib/libcmocka.a)

