IF(LEVELDB_FOUND)

INCLUDE_DIRECTORIES( ${LEVELDB_INCLUDE_DIR} )

SET(TARGET_H
    LevelDBCacheOptions
    LevelDBCache
    LevelDBCacheBin
	Tracker
)
SET(TARGET_SRC 
    LevelDBCache.cpp
    LevelDBCacheBin.cpp
    LevelDBCacheDriver.cpp
)

SET(TARGET_LIBRARIES_VARS LEVELDB_LIBRARY)

if(WIN32)
	SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} Shlwapi.lib)
endif()

SETUP_PLUGIN(osgearth_cache_leveldb)


# to install public driver includes:
SET(LIB_NAME cache_leveldb)
SET(LIB_PUBLIC_HEADERS LevelDBCacheOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

ENDIF(LEVELDB_FOUND)

