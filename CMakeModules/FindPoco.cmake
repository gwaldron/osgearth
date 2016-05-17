# Locate Poco libraries.
# This module defines
# POCO_NET_LIBRARY
# POCO_FOUNDATION_LIBRARY
# POCO_FOUND, if false, do not try to link to Poco
# POCO_INCLUDE_DIR, where to find the headers

SET(POCO_DIR "" CACHE PATH "Set to base Poco install path")

FIND_PATH(POCO_INCLUDE_DIR Poco/Poco.h
    ${POCO_DIR}/include
    $ENV{POCO_DIR}/include
    $ENV{POCO_DIR}/Source/lib #Windows Binary Installer
    $ENV{POCO_DIR}
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


MACRO( FIND_POCO_LIBRARY MYLIBRARY MYLIBRARYNAME )

FIND_LIBRARY(${MYLIBRARY}
    NAMES
        ${MYLIBRARYNAME}
    PATHS
        ${POCO_DIR}
        $ENV{POCO_DIR}
        $ENV{POCO_DIR}
        ~/Library/Frameworks
        /Library/Frameworks
        /usr/local
        /usr
        /sw
        /opt/local
        /opt/csw
        /opt
        /usr/freeware
    PATH_SUFFIXES
        /lib/
        /lib64/
        /build/lib/
        /build/lib64/
        /Build/lib/
        /Build/lib64/
     )

ENDMACRO(FIND_POCO_LIBRARY LIBRARY LIBRARYNAME)

FIND_POCO_LIBRARY(POCO_FOUNDATION_LIBRARY PocoFoundation)
FIND_POCO_LIBRARY(POCO_FOUNDATION_LIBRARY_DEBUG PocoFoundationd)
FIND_POCO_LIBRARY(POCO_NET_LIBRARY PocoNet)
FIND_POCO_LIBRARY(POCO_NET_LIBRARY_DEBUG PocoNetd)
FIND_POCO_LIBRARY(POCO_UTIL_LIBRARY PocoUtil)
FIND_POCO_LIBRARY(POCO_UTIL_LIBRARY_DEBUG PocoUtild)

SET(POCO_FOUND "NO")
IF(POCO_FOUNDATION_LIBRARY AND POCO_NET_LIBRARY AND POCO_UTIL_LIBRARY AND POCO_INCLUDE_DIR)
    SET(POCO_FOUND "YES")
ENDIF()


