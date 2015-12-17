project(props)
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

set(props_HEADERS
)

set(props_SOURCES
    ${props_HEADERS}
    main.cpp
)

add_executable(props ${props_SOURCES})

target_link_libraries(props
    ${CS2_LIBRARIES}
)
