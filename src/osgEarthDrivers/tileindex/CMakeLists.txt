SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology osgEarthUtil)

SET(TARGET_SRC
    ReaderWriterTileIndex.cpp
)
SET(TARGET_H
    TileIndexOptions
)

SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OPENTHREADS_LIBRARY) 
SETUP_PLUGIN(osgearth_tileindex)

# to install public driver includes:
SET(LIB_NAME tileindex)
SET(LIB_PUBLIC_HEADERS TileIndexOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
