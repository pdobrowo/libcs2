#
# Try to find the GMP libraries
#
# GMP_FOUND - GMP found
# GMP_INCLUDE_DIR - GMP include directory
# GMP_LIBRARIES - GMP shared libraries
#
if (GMP_INCLUDE_DIR AND GMP_LIBRARIES)
    set(GMP_FIND_QUIETLY TRUE)
endif (GMP_INCLUDE_DIR AND GMP_LIBRARIES)

find_path(GMP_INCLUDE_DIR
          NAMES gmp.h
          PATHS /usr/include/gmp
                /usr/local/include/gmp)

find_library(GMP_LIBRARIES
             NAMES gmp libgmp)

message(STATUS "GMP_INCLUDE_DIR: ${GMP_INCLUDE_DIR}")
message(STATUS "GMP_LIBRARIES: ${GMP_LIBRARIES}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GMP DEFAULT_MSG GMP_INCLUDE_DIR GMP_LIBRARIES)

mark_as_advanced(GMP_INCLUDE_DIR GMP_LIBRARIES)