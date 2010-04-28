# Locate GEOS.
# This module defines
# GEOS_LIBRARY
# GEOS_LIBRARY_DEBUG
# GEOS_FOUND, if false, do not try to link to geos
# GEOS_INCLUDE_DIR, where to find the headers

FIND_PATH(GEOS_INCLUDE_DIR geos.h
  $ENV{GDAL_DIR}
  NO_DEFAULT_PATH
    PATH_SUFFIXES include
)

FIND_PATH(GEOS_INCLUDE_DIR geos.h
  PATHS
  ~/Library/Frameworks/geos/Headers
  /Library/Frameworks/geos/Headers
  /usr/local/include/geos
  /usr/local/include/GEOS
  /usr/local/include
  /usr/include/geos
  /usr/include/GEOS
  /usr/include
  /sw/include/geos 
  /sw/include/GEOS 
  /sw/include # Fink
  /opt/local/include/geos
  /opt/local/include/GEOS
  /opt/local/include # DarwinPorts
  /opt/csw/include/geos
  /opt/csw/include/GEOS
  /opt/csw/include # Blastwave
  /opt/include/geos
  /opt/include/GEOS
  /opt/include
  e:/devel/geos-3.1.1/source/headers
)

FIND_LIBRARY(GEOS_LIBRARY
  NAMES geos
  PATHS
    $ENV{GEOS_DIR}
    NO_DEFAULT_PATH
    PATH_SUFFIXES lib64 lib
)

FIND_LIBRARY(GEOS_LIBRARY
  NAMES geos
  PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local
    /usr
    /sw
    /opt/local
    /opt/csw
    /opt
    /usr/freeware    
  PATH_SUFFIXES lib64 lib
)


FIND_LIBRARY(GEOS_LIBRARY_DEBUG
  NAMES geod_d geos_i_d geosd
  PATHS
    $ENV{GEOS_DIR}
    NO_DEFAULT_PATH
    PATH_SUFFIXES lib64 lib
)

FIND_LIBRARY(GEOS_LIBRARY_DEBUG
  NAMES geod_d geos_i_d geosd
  PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local
    /usr
    /sw
    /opt/local
    /opt/csw
    /opt
    /usr/freeware    
  PATH_SUFFIXES lib64 lib
)

SET(GEOS_FOUND "NO")
IF(GEOS_LIBRARY AND GEOS_INCLUDE_DIR)
  SET(GEOS_FOUND "YES")
ENDIF(GEOS_LIBRARY AND GEOS_INCLUDE_DIR)

