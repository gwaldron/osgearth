IF(ROCKSDB_FOUND OR RocksDB_FOUND)

SET(CMAKE_CXX_STANDARD 11)

INCLUDE_DIRECTORIES( ${ROCKSDB_INCLUDE_DIR} )

SET(TARGET_H
    RocksDBCacheOptions
    RocksDBCache
    RocksDBCacheBin
    Tracker
)
SET(TARGET_SRC 
    RocksDBCache.cpp
    RocksDBCacheBin.cpp
    RocksDBCacheDriver.cpp
)

SET(TARGET_LIBRARIES_VARS ROCKSDB_LIBRARY ZLIB_LIBRARY)

IF(MSVC)
    SET(TARGET_EXTERNAL_LIBRARIES ws2_32 winmm rpcrt4 shlwapi)
ENDIF(MSVC)

IF(MINGW)
    SET(TARGET_EXTERNAL_LIBRARIES RocksDB::rocksdb)
ENDIF(MINGW)

SETUP_PLUGIN(osgearth_cache_rocksdb)

# to install public driver includes:
SET(LIB_NAME cache_rocksdb)
SET(LIB_PUBLIC_HEADERS RocksDBCacheOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

ENDIF(ROCKSDB_FOUND OR RocksDB_FOUND)
