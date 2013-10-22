# Locate JavaScriptCore
# This module defines
# JAVASCRIPTCORE_LIBRARY
# JAVASCRIPTCORE_FOUND, if false, do not try to link to JavaScriptCore
# JAVASCRIPTCORE_INCLUDE_DIR, where to find the headers

FIND_PATH(JAVASCRIPTCORE_INCLUDE_DIR JavaScriptCore.h
    ${JAVASCRIPTCORE_DIR}/include
    $ENV{JAVASCRIPTCORE_DIR}/include
    $ENV{JAVASCRIPTCORE_DIR}
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/include
    /usr/include
    /sw/include # Fink
    /opt/local/include # DarwinPorts
    /opt/csw/include # Blastwave
    /opt/include
    /usr/freeware/include
    /devel
)

FIND_LIBRARY(JAVASCRIPTCORE_LIBRARY
    NAMES libJavaScriptCore
    PATHS
    ${JAVASCRIPTCORE_DIR}
    ${JAVASCRIPTCORE_DIR}/lib
    $ENV{JAVASCRIPTCORE_DIR}
    $ENV{JAVASCRIPTCORE_DIR}/lib
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/lib
    /sw/lib
    /opt/local/lib
    /opt/csw/lib
    /opt/lib
    /usr/freeware/lib64
)

SET(JAVASCRIPTCORE_FOUND "NO")
IF(JAVASCRIPTCORE_LIBRARY AND JAVASCRIPTCORE_INCLUDE_DIR)
    SET(JAVASCRIPTCORE_FOUND "YES")
ENDIF(JAVASCRIPTCORE_LIBRARY AND JAVASCRIPTCORE_INCLUDE_DIR)


