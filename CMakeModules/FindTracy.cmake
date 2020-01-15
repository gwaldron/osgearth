# Locate Tracy profiler shared library.
# This module defines
# TRACY_LIBRARY
# TRACY_FOUND, if false, do not try to link to tracy
# TRACY_INCLUDE_DIR, where to find the headers

SET(TRACY_DIR "" CACHE PATH "Root directory of Tracy distribution")

FIND_PATH(TRACY_INCLUDE_DIR Tracy.hpp
  PATHS
    ${TRACY_DIR}
    $ENV{TRACY_DIR}
  PATH_SUFFIXES include
)

FIND_LIBRARY(TRACY_LIBRARY
  NAMES tracy TracyClient
  PATHS  
    ${TRACY_DIR}/lib
    $ENV{TRACY_DIR}
  PATH_SUFFIXES lib64 lib
)

SET(TRACY_FOUND "NO")
IF(TRACY_LIBRARY AND TRACY_INCLUDE_DIR)
  SET(TRACY_FOUND "YES")
ENDIF(TRACY_LIBRARY AND TRACY_INCLUDE_DIR)