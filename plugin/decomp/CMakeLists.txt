###
 # CS2 library
 # plugin: decomp
 #
 # Copyright 2015 Przemyslaw Dobrowolski <przemyslaw.dobrowolski@xtb.com>
 #
 ##
project(decomp)
cmake_minimum_required(VERSION 2.8.12)

# modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/../../cmake)

# cs2
find_package(CS2 REQUIRED)
include_directories(${CS2_INCLUDE_DIR})
link_directories(${CS2_LIBRARY_DIR})

# CGAL
find_package(CGAL REQUIRED)
include(${CGAL_USE_FILE})

# compiler settings
add_definitions(
    -Wall
    -Wextra
    -pedantic
    -ansi
    -Wno-long-long
)

# setup
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")

# optimization
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fomit-frame-pointer -mtune=native")

# output directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/../../bin)

# internal
include_directories(${CMAKE_SOURCE_DIR}/inc)

# inc
set(decomp_HEADERS
    decomp3f.h
)

# src
set(decomp_SOURCES
    ${decomp_HEADERS}
    decomp3f.cpp
)

add_library(decomp SHARED ${decomp_SOURCES})

target_link_libraries(decomp
    ${CS2_LIBRARIES}
)

message("CS2 system: ${CMAKE_SYSTEM_NAME}")
message("CS2 system version: ${CMAKE_SYSTEM_VERSION}")
message("CS2 compiler: ${CMAKE_CXX_COMPILER_ID}")
message("CS2 processor: ${CMAKE_SYSTEM_PROCESSOR}")
