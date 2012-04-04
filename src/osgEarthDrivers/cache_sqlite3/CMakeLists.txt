
INCLUDE_DIRECTORIES( ${SQLITE3_INCLUDE_DIR} )

#SET(TARGET_COMMON_LIBRARIES
#    ${TARGET_COMMON_LIBRARIES}
#    ${SQLITE3_LIBRARY}
#)

SET(TARGET_H
    Sqlite3CacheOptions
)
SET(TARGET_SRC 
    Sqlite3Cache.cpp
)

SET(TARGET_LIBRARIES_VARS SQLITE3_LIBRARY)

SETUP_PLUGIN(osgearth_cache_sqlite3)


# to install public driver includes:
SET(LIB_NAME cache_sqlite3)
SET(LIB_PUBLIC_HEADERS Sqlite3CacheOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

