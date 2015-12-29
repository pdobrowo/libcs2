project(exact)
cmake_minimum_required(VERSION 2.6)

# modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/../../cmake)

# cs2
find_package(CS2 REQUIRED)
include_directories(${CS2_INCLUDE_DIR})
link_directories(${CS2_LIBRARY_DIR})

# output directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/../../bin)

# compiler settings
add_definitions(
    -Wall
    -Wextra
    -pedantic
    -Wno-long-long
    -Wno-variadic-macros
)

# optimizations
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -ffast-math -fomit-frame-pointer -march=native")

set(exact_HEADERS
)

set(exact_SOURCES
    ${exact_HEADERS}
    main.cpp
)

add_executable(exact ${exact_SOURCES})

target_link_libraries(exact
    ${CS2_LIBRARIES}
)
