# Locate leveldb.
# This module defines
# LEVELDB_LIBRARY
# LEVELDB_FOUND, if false, do not try to link to libnoise
# LEVELDB_INCLUDE_DIR, where to find the headers

FIND_PATH(LEVELDB_INCLUDE_DIR leveldb/db.h
  $ENV{LEVELDB_DIR}
  NO_DEFAULT_PATH
    PATH_SUFFIXES include
)

FIND_PATH(LEVELDB_INCLUDE_DIR leveldb/db.h
  PATHS
  ~/Library/Frameworks/noise/Headers
  /Library/Frameworks/noise/Headers
  /usr/local/include/leveldb
  /usr/local/include/leveldb
  /usr/local/include
  /usr/include/leveldb
  /usr/include/leveldb
  /usr/include
  /sw/include/leveldb 
  /sw/include/leveldb 
  /sw/include # Fink
  /opt/local/include/leveldb
  /opt/local/include/leveldb
  /opt/local/include # DarwinPorts
  /opt/csw/include/leveldb
  /opt/csw/include/leveldb
  /opt/csw/include # Blastwave
  /opt/include/leveldb
  /opt/include/leveldb
  /opt/include  
)

FIND_LIBRARY(LEVELDB_LIBRARY
  NAMES libleveldb leveldb leveldb_static
  PATHS
    $ENV{LEVELDB_DIR}
    NO_DEFAULT_PATH
    PATH_SUFFIXES lib64 lib
)

FIND_LIBRARY(LEVELDB_LIBRARY
  NAMES libleveldb leveldb leveldb_static
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
FIND_LIBRARY(LEVELDB_LIBRARY_DEBUG
  NAMES libleveldbd leveldbd leveldb_staticd
  PATHS
    $ENV{LEVELDB_DIR}
    NO_DEFAULT_PATH
    PATH_SUFFIXES lib64 lib
)

FIND_LIBRARY(LEVELDB_LIBRARY_DEBUG
  NAMES libleveldbd leveldbd leveldb_staticd
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

SET(LEVELDB_FOUND "NO")
IF(LEVELDB_LIBRARY AND LEVELDB_INCLUDE_DIR)
  SET(LEVELDB_FOUND "YES")
ENDIF(LEVELDB_LIBRARY AND LEVELDB_INCLUDE_DIR)

