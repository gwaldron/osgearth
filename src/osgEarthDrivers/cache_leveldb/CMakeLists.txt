
INCLUDE_DIRECTORIES( ${LEVELDB_INCLUDE_DIR} )

SET(TARGET_H
    LevelDBCache
)
SET(TARGET_SRC 
    LevelDBCache.cpp
)

SET(TARGET_LIBRARIES_VARS LEVELDB_LIBRARY)

SETUP_PLUGIN(osgearth_cache_leveldb)


# to install public driver includes:
SET(LIB_NAME cache_leveldb)
SET(LIB_PUBLIC_HEADERS LevelDBCache)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

