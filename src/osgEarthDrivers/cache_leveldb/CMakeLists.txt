FIND_PACKAGE(LevelDB)

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

    SET(TARGET_LIBRARIES_VARS LEVELDB_LIBRARY ZLIB_LIBRARY OSG_LIBRARY OSGDB_LIBRARY OPENTHREADS_LIBRARY)

    IF(MSVC)
        SET(TARGET_EXTERNAL_LIBRARIES Shlwapi)
    ENDIF(MSVC)

    SETUP_PLUGIN(osgearth_cache_leveldb)


    # to install public driver includes:
    SET(LIB_NAME cache_leveldb)
    SET(LIB_PUBLIC_HEADERS LevelDBCacheOptions)
    INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

ENDIF(LEVELDB_FOUND)
