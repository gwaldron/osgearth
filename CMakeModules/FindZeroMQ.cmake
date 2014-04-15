# Locate zeromq
# This module defines
# ZEROMQ_LIBRARY
# ZEROMQ_FOUND, if false, do not try to link to zeromq
# ZEROMQ_INCLUDE_DIR, where to find the headers

FIND_PATH(ZEROMQ_INCLUDE_DIR zmq.h
  $ENV{ZEROMQ_DIR}
  NO_DEFAULT_PATH
    PATH_SUFFIXES include
)

FIND_PATH(ZEROMQ_INCLUDE_DIR zmq.h
  PATHS
  ~/Library/Frameworks/zeromq/Headers
  /Library/Frameworks/zeromq/Headers
  /usr/local/include/zeromq
  /usr/local/include/zeromq
  /usr/local/include
  /usr/include/zeromq
  /usr/include/zeromq
  /usr/include
  /sw/include/zeromq 
  /sw/include/zeromq 
  /sw/include # Fink
  /opt/local/include/zeromq
  /opt/local/include/zeromq
  /opt/local/include # DarwinPorts
  /opt/csw/include/zeromq
  /opt/csw/include/zeromq
  /opt/csw/zeromq # Blastwave
  /opt/include/zeromq
  /opt/include/zeromq
  /opt/include  
)

FIND_LIBRARY(ZEROMQ_LIBRARY
  NAMES libzmq libzmq libzmq
  PATHS
    $ENV{ZEROMQ_DIR}
    NO_DEFAULT_PATH
    PATH_SUFFIXES lib64 lib
)

FIND_LIBRARY(ZEROMQ_LIBRARY
  NAMES libzmq
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

SET(ZEROMQ_FOUND "NO")
IF(ZEROMQ_LIBRARY AND ZEROMQ_INCLUDE_DIR)
  SET(ZEROMQ_FOUND "YES")
ENDIF(ZEROMQ_LIBRARY AND ZEROMQ_INCLUDE_DIR)

