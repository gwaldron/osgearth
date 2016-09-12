IF(ROCKSDB_FOUND)

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

SET(TARGET_LIBRARIES_VARS ROCKSDB_LIBRARY)

IF(WIN32)
    SET(TARGET_EXTERNAL_LIBRARIES ws2_32 winmm rpcrt4)
ENDIF(WIN32)

SETUP_PLUGIN(osgearth_cache_rocksdb)

# to install public driver includes:
SET(LIB_NAME cache_rocksdb)
SET(LIB_PUBLIC_HEADERS RocksDBCacheOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

ENDIF(ROCKSDB_FOUND)


