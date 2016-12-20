###
 # CS2 library
 #
 # Copyright 2015 Przemyslaw Dobrowolski <przemyslaw.dobrowolski@xtb.com>
 #
 ##
project(cs2)
cmake_minimum_required(VERSION 2.6)

include(CheckCCompilerFlag)

# modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# gmp
find_package(GMP REQUIRED)
include_directories(${GMP_INCLUDE_DIR})

# compiler settings
add_definitions(
    -Wall
    -Wextra
    -pedantic
    -Wno-long-long
)

# optimizations
check_c_compiler_flag(-Ofast COMPILER_SUPPORT_OFAST)

if(COMPILER_SUPPORT_OFAST)
    set(COMPILER_RELEASE_OPTIMIZATION -Ofast)
else(COMPILER_SUPPORT_OFAST)
    set(COMPILER_RELEASE_OPTIMIZATION -O3)
endif(COMPILER_SUPPORT_OFAST)

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${COMPILER_RELEASE_OPTIMIZATION} -fomit-frame-pointer -march=native")

# output directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

# internal
include_directories(${CMAKE_SOURCE_DIR}/inc)

# inc
set(cs2_HEADERS
    # floating-point arithmetic
    inc/cs2/vec3f.h
    inc/cs2/vec4f.h
    inc/cs2/mat44f.h
    inc/cs2/plane3f.h
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
)

# src
set(cs2_SOURCES
    ${cs2_HEADERS}

    # floating-point arithmetic
    src/vec3f.c
    src/vec4f.c
    src/mat44f.c
    src/plane3f.c
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
)

add_library(cs2 SHARED ${cs2_SOURCES})
add_library(cs2_s STATIC ${cs2_SOURCES})

set_target_properties(cs2 PROPERTIES C_VISIBILITY_PRESET hidden)
target_link_libraries(cs2 c m ${GMP_LIBRARIES})

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_link_libraries(cs2 dl)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

message("CS2 system: ${CMAKE_SYSTEM_NAME}")
message("CS2 system version: ${CMAKE_SYSTEM_VERSION}")
message("CS2 compiler: ${CMAKE_C_COMPILER_ID}")
message("CS2 processor: ${CMAKE_SYSTEM_PROCESSOR}")
