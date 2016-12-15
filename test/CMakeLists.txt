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

# cs2
find_package(CS2 REQUIRED)
include_directories(${CS2_INCLUDE_DIR})
link_directories(${CS2_LIBRARY_DIR})

# criterion
include_directories(${CMAKE_SOURCE_DIR}/../deps/criterion/include)
link_directories(${CMAKE_SOURCE_DIR}/../deps/criterion)

# output directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/../bin)

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

set(test_HEADERS
)

set(unittest_SOURCES
    ${unittest_HEADERS}
    vec3f.c
    vec3x.c
)

add_executable(unittest ${unittest_SOURCES})

target_link_libraries(unittest
    ${CS2_LIBRARIES}
    criterion
)
