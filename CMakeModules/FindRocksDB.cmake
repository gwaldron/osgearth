# Locate RocksDB

FIND_PATH(ROCKSDB_INCLUDE_DIR rocksdb/db.h
  PATHS
  ${ROCKSDB_DIR}
  NO_DEFAULT_PATH
  PATH_SUFFIXES include
)

find_library(ROCKSDB_LIBRARY NAMES ROCKSDBLIB
             PATHS
               ${ROCKSDB_DIR}
               ${ROCKSDB_DIR}/bin/Release
               ${ROCKSDB_DIR}/bin64_vs2013/Release
             PATH_SUFFIXES lib lib64
	    )


find_library(ROCKSDB_LIBRARY_DEBUG NAMES ROCKSDBLIB
             PATHS
               ${ROCKSDB_DIR}
               ${ROCKSDB_DIR}/bin/Debug
               ${ROCKSDB_DIR}/bin64_vs2013/Debug
             PATH_SUFFIXES lib lib64
	    )

if( ROCKSDB_LIBRARY )
  MESSAGE(STATUS "FOUND ROCKSDB_LIBRARY")
endif()

if (ROCKSDB_INCLUDE_DIR)
  MESSAGE(STATUS "FOUND ROCKSDB_INCLUDE_DIR")
endif()
  

set( ROCKSDB_FOUND "NO" )
if( ROCKSDB_LIBRARY AND ROCKSDB_INCLUDE_DIR )
    set( ROCKSDB_FOUND "YES" )
    MESSAGE(STATUS "Found RocksDB library: " ${ROCKSDB_LIBRARY})
else()
    MESSAGE(STATUS "Could not find ROCKSDB")
endif()
