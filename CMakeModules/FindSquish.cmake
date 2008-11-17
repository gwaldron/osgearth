# Locate EXPAT
# This module defines
# SQUISH_LIBRARY, SQUISH_LIBRARY_DEBUG
# SQUISH_FOUND, if false, do not try to link to expat
# SQUISH_INCLUDE_DIR, where to find the headers
#
# Created by Robert Osfield, adapted by GW

FIND_PATH(SQUISH_INCLUDE_DIR squish.h
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

FIND_LIBRARY(SQUISH_LIBRARY
    NAMES squish libsquish
    PATHS
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

FIND_LIBRARY(SQUISH_LIBRARY_DEBUG
    NAMES squishd libsquishd
    PATHS
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

IF( SQUISH_LIBRARY AND SQUISH_LIBRARY_DEBUG AND SQUISH_INCLUDE )
    SET( SQUISH_FOUND "YES" )
ENDIF( SQUISH_LIBRARY AND SQUISH_LIBRARY_DEBUG AND SQUISH_INCLUDE )

