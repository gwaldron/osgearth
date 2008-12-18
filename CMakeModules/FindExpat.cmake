# Locate EXPAT
# This module defines
# EXPAT_LIBRARY
# EXPAT_FOUND, if false, do not try to link to expat
# EXPAT_INCLUDE_DIR, where to find the headers
#
# $EXPAT_DIR is an environment variable that would
# correspond to the ./configure --prefix=$EXPAT_DIR
#
# Created by Robert Osfield, adapted by GW

FIND_PATH(EXPAT_INCLUDE_DIR expat.h
    ${EXPAT_DIR}/include
    $ENV{EXPAT_DIR}/include
	$ENV{EXPAT_DIR}/Source/lib #Windows Binary Installer
    $ENV{EXPAT_DIR}
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

FIND_LIBRARY(EXPAT_LIBRARY
    NAMES expat expat_i expat-2.0.1 libexpat
    PATHS
    ${EXPAT_DIR}/lib
    $ENV{EXPAT_DIR}/lib
	$ENV{EXPAT_DIR}/bin #Windows Binary Installer
    $ENV{EXPAT_DIR}
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

SET(EXPAT_FOUND "NO")
IF(EXPAT_LIBRARY AND EXPAT_INCLUDE_DIR)
    SET(EXPAT_FOUND "YES")
ENDIF(EXPAT_LIBRARY AND EXPAT_INCLUDE_DIR)


