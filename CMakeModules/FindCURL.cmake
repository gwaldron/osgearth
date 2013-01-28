FIND_PATH(CURL_INCLUDE_DIR curl.h curl/curl.h
  $ENV{CURL_DIR}
  NO_DEFAULT_PATH
    PATH_SUFFIXES include
)

FIND_PATH(CURL_INCLUDE_DIR curl.h curl/curl.h
    PATHS ${CMAKE_PREFIX_PATH} # Unofficial: We are proposing this.
    NO_DEFAULT_PATH
    PATH_SUFFIXES include
)

FIND_PATH(CURL_INCLUDE_DIR curl.h curl/curl.h
  PATHS
  ${THIRD_PARTY_DIR}/include
  /usr/local/include/curl
  /usr/local/include/CURL
  /usr/local/include
  /usr/include/curl
  /usr/include/CURL
  /usr/include
  /sw/include/curl
  /sw/include/CURL
  /sw/include # Fink
  /opt/local/include/curl
  /opt/local/include/CURL
  /opt/local/include # DarwinPorts
  /opt/csw/include/curl
  /opt/csw/include/CURL
  /opt/csw/include # Blastwave
  /opt/include/curl
  /opt/include/CURL
  /opt/include
)

#Find the CURL release library
FIND_LIBRARY(CURL_LIBRARY
  NAMES curl curllib CURL libcurl
  PATHS
  ${THIRD_PARTY_DIR}/lib
  $ENV{CURL_DIR}
  NO_DEFAULT_PATH
  PATH_SUFFIXES lib64 lib
)
FIND_LIBRARY(CURL_LIBRARY
  NAMES curl CURL curllib libcurl
  PATHS ${CMAKE_PREFIX_PATH} # Unofficial: We are proposing this.
    NO_DEFAULT_PATH
    PATH_SUFFIXES lib64 lib
)
FIND_LIBRARY(CURL_LIBRARY
  NAMES curl CURL curllib libcurl
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
    [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;CURL_ROOT]/lib
  PATH_SUFFIXES lib64 lib
)

#Find the CURL debug library
FIND_LIBRARY(CURL_LIBRARY_DEBUG
  NAMES curlD curld curllibD curllibd CURLD libcurlD libcurld
  PATHS
  ${THIRD_PARTY_DIR}/lib
  $ENV{CURL_DIR}
  NO_DEFAULT_PATH
  PATH_SUFFIXES lib64 lib
)
FIND_LIBRARY(CURL_LIBRARY_DEBUG
  NAMES curlD curld curllibD curllibd CURLD libcurlD libcurld
  PATHS ${CMAKE_PREFIX_PATH} # Unofficial: We are proposing this.
    NO_DEFAULT_PATH
    PATH_SUFFIXES lib64 lib
)
FIND_LIBRARY(CURL_LIBRARY_DEBUG
  NAMES curlD curld curllibD curllibd CURLD libcurlD libcurld
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
    [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;CURL_ROOT]/lib
  PATH_SUFFIXES lib64 lib
)

SET(CURL_FOUND "NO")
IF(CURL_LIBRARY AND CURL_INCLUDE_DIR)
  SET(CURL_FOUND "YES")
ENDIF(CURL_LIBRARY AND CURL_INCLUDE_DIR)

