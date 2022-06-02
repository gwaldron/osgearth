# Locate RocksDB
# - Find RocksDB
#
#  ROCKSDB_INCLUDE_DIR - Where to find rocksdb/db.h
#  ROCKSDB_LIBRARY     - List of libraries when using ROCKSDB.
#  ROCKSDB_FOUND      - True if ROCKSDB found.

get_filename_component(module_file_path ${CMAKE_CURRENT_LIST_FILE} PATH)

# Look for the header file.
find_path(ROCKSDB_INCLUDE_DIR NAMES rocksdb/db.h PATHS $ENV{ROCKSDB_ROOT}/include /opt/local/include /usr/local/include /usr/include    DOC "Path in which the file rocksdb/db.h is located." )
mark_as_advanced(ROCKSDB_INCLUDE_DIR)

# Look for the library.
# Does this work on UNIX systems? (LINUX)
find_library(ROCKSDB_LIBRARY NAMES rocksdb PATHS /usr/lib $ENV{ROCKSDB_ROOT}/lib DOC "Path to rocksdb library." )
mark_as_advanced(ROCKSDB_LIBRARY)

# Copy the results to the output variables.
if (ROCKSDB_INCLUDE_DIR AND ROCKSDB_LIBRARY)
  message(STATUS "Found rocksdb in ${ROCKSDB_INCLUDE_DIR} ${ROCKSDB_LIBRARY}")
  set(ROCKSDB_FOUND 1)
  include(CheckCXXSourceCompiles)
  #set(CMAKE_REQUIRED_LIBRARY ${ROCKSDB_LIBRARY} pthread)
  set(CMAKE_REQUIRED_INCLUDES ${ROCKSDB_INCLUDE_DIR})
  find_package(ZLIB REQUIRED)
else ()
  set(ROCKSDB_FOUND 0)
endif ()

 # Report the results.
if (NOT ROCKSDB_FOUND AND NOT QUIET)
   set(ROCKSDB_DIR_MESSAGE "ROCKSDB was not found. Make sure ROCKSDB_LIBRARY and ROCKSDB_INCLUDE_DIR are set.")
   if (ROCKSDB_FIND_REQUIRED)
     message(FATAL_ERROR "${ROCKSDB_DIR_MESSAGE}")
   elseif (NOT ROCKSDB_FIND_QUIETLY)
     message(STATUS "${ROCKSDB_DIR_MESSAGE}")
   endif ()
endif ()

