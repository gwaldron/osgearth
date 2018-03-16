# Locate RocksDB

SET(ROCKSDB_DIR "" CACHE PATH "Root directory of RocksDB distribution")

FIND_PATH(ROCKSDB_INCLUDE_DIR rocksdb/db.h
  PATHS
  ${ROCKSDB_DIR}
)

if(RocksDB_FIND_REQUIRED_STATIC)
	set(ROCKSDB_LIB "rocksdblib")
else()
	set(ROCKSDB_LIB "rocksdb")
endif()

find_library(ROCKSDB_LIBRARY NAMES ${ROCKSDB_LIB}
			 PATHS
			   ${ROCKSDB_DIR}
			   ${ROCKSDB_DIR}/bin/Release
			   ${ROCKSDB_DIR}/bin64_vs2013/Release
			 PATH_SUFFIXES lib lib64
		)


find_library(ROCKSDB_LIBRARY_DEBUG NAMES ${ROCKSDB_LIB}d ${ROCKSDB_LIB}
			 PATHS
			   ${ROCKSDB_DIR}
			   ${ROCKSDB_DIR}/bin/Debug
			   ${ROCKSDB_DIR}/bin64_vs2013/Debug
			 PATH_SUFFIXES lib lib64
			)

find_package_handle_standard_args(ROCKSDB
    FOUND_VAR
      ROCKSDB_FOUND
    REQUIRED_VARS
      ROCKSDB_LIBRARY
      ROCKSDB_INCLUDE_DIR
    FAIL_MESSAGE
      "Could NOT find ROCKSDB"
)

set(ROCKSDB_INCLUDE_DIRS ${ROCKSDB_INCLUDE_DIR} )
set(ROCKSDB_LIBRARIES ${ROCKSDB_LIBRARY})
