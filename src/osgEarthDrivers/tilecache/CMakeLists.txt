

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES})

SET(TARGET_SRC ReaderWriterTileCache.cpp)

SET(TARGET_H TileCacheOptions)

SETUP_PLUGIN(osgearth_tilecache)



# to install public driver includes:
SET(LIB_NAME tilecache)
SET(LIB_PUBLIC_HEADERS TileCacheOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

