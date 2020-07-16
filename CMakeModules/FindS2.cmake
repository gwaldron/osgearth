# Locate Google's S2 Library
# This module defines
# S2_LIBRARY
# S2_FOUND, if false, do not try to link to S2
# S2_INCLUDE_DIR, where to find the headers

SET(S2_DIR "" CACHE PATH "Root directory of S2 distribution")

FIND_PATH(S2_INCLUDE_DIR s2/s2cell.h
  PATHS
    ${S2_DIR}
    $ENV{S2_DIR}
  PATH_SUFFIXES include
)

FIND_LIBRARY(S2_LIBRARY
  NAMES s2
  PATHS
    ${S2_DIR}/lib
    $ENV{S2_DIR}
  PATH_SUFFIXES lib64 lib
)

SET(S2_FOUND "NO")
IF(S2_LIBRARY AND S2_INCLUDE_DIR)
  SET(S2_FOUND "YES")
ENDIF(S2_LIBRARY AND S2_INCLUDE_DIR)
