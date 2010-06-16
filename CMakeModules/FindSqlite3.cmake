# Locate SQLITE3
# This module defines
# SQLITE3_LIBRARY
# SQLITE3_FOUND, if false, do not try to link to expat
# SQLITE3_INCLUDE_DIR, where to find the headers

FIND_PATH(SQLITE3_INCLUDE_DIR sqlite3.h
    ${SQLITE3_DIR}/include/sqlite
    $ENV{SQLITE3_DIR}/include/sqlite
    $ENV{SQLITE3_DIR}/Source/lib/sqlite #Windows Binary Installer
    $ENV{SQLITE3_DIR}/sqlite
    ~/Library/Frameworks/sqlite
    /Library/Frameworks/sqlite
    /usr/local/include
    /usr/include
    /sw/include # Fink
    /opt/local/include # DarwinPorts
    /opt/csw/include # Blastwave
    /opt/include
    /usr/freeware/include
    /devel
)

FIND_LIBRARY(SQLITE3_LIBRARY
    NAMES sqlite3 libsqlite3
    PATHS
    ${SQLITE3_DIR}/lib
    $ENV{SQLITE3_DIR}/lib
    $ENV{SQLITE3_DIR}/bin #Windows Binary Installer
    $ENV{SQLITE3_DIR}
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

SET(SQLITE3_FOUND "NO")
IF(SQLITE3_LIBRARY AND SQLITE3_INCLUDE_DIR)
    SET(SQLITE3_FOUND "YES")
ENDIF(SQLITE3_LIBRARY AND SQLITE3_INCLUDE_DIR)


