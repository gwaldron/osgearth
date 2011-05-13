SET(TARGET_SRC
    ReaderWriterTileService.cpp
)
SET(TARGET_H
    TileServiceOptions
)

SETUP_PLUGIN(osgearth_tileservice)


# to install public driver includes:
SET(LIB_NAME tileservice)
SET(LIB_PUBLIC_HEADERS TileServiceOptions)
INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} )
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

