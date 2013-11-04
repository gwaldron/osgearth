# Locate V8
# This module defines
# V8_LIBRARY
# V8_FOUND, if false, do not try to link to V8
# V8_INCLUDE_DIR, where to find the headers

FIND_PATH(V8_INCLUDE_DIR v8.h
    ${V8_DIR}/include
    $ENV{V8_DIR}/include
    $ENV{V8_DIR}
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

FIND_LIBRARY(V8_BASE_LIBRARY
    NAMES v8_base v8_base.ia32 libv8_base
    PATHS
    ${V8_DIR}
    ${V8_DIR}/lib
    ${V8_DIR}/build/Release/lib
    $ENV{V8_DIR}
    $ENV{V8_DIR}/lib
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

FIND_LIBRARY(V8_SNAPSHOT_LIBRARY
    NAMES v8_snapshot libv8_snapshot
    PATHS
    ${V8_DIR}
    ${V8_DIR}/lib
    ${V8_DIR}/build/Release/lib
    $ENV{V8_DIR}
    $ENV{V8_DIR}/lib
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

FIND_LIBRARY(V8_ICUUC_LIBRARY
    NAMES icuuc libicuuc
    PATHS
    ${V8_DIR}
    ${V8_DIR}/lib
    ${V8_DIR}/build/Release/lib
    $ENV{V8_DIR}
    $ENV{V8_DIR}/lib
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

FIND_LIBRARY(V8_ICUI18N_LIBRARY
    NAMES icui18n libicui18n
    PATHS
    ${V8_DIR}
    ${V8_DIR}/lib
    ${V8_DIR}/build/Release/lib
    $ENV{V8_DIR}
    $ENV{V8_DIR}/lib
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

SET(V8_FOUND "NO")
IF(V8_BASE_LIBRARY AND V8_SNAPSHOT_LIBRARY AND V8_ICUUC_LIBRARY AND V8_ICUI18N_LIBRARY AND V8_INCLUDE_DIR)
    SET(V8_FOUND "YES")
ENDIF(V8_BASE_LIBRARY AND V8_SNAPSHOT_LIBRARY AND V8_ICUUC_LIBRARY AND V8_ICUI18N_LIBRARY AND V8_INCLUDE_DIR)


