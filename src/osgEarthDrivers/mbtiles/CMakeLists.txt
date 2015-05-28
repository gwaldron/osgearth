IF(SQLITE3_FOUND)

INCLUDE_DIRECTORIES( ${SQLITE3_INCLUDE_DIR} )

#SET(TARGET_COMMON_LIBRARIES
#    ${TARGET_COMMON_LIBRARIES}
#    ${SQLITE3_LIBRARY}
#)

SET(TARGET_SRC
    MBTilesPlugin.cpp
	MBTilesTileSource.cpp
)

# headers to show in IDE
SET(TARGET_H    
    MBTilesOptions
	MBTilesTileSource
)

SET(TARGET_LIBRARIES_VARS SQLITE3_LIBRARY)

SETUP_PLUGIN(osgearth_mbtiles)


# to install public driver includes:
SET(LIB_NAME mbtiles)
SET(LIB_PUBLIC_HEADERS MBTilesOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

ENDIF(SQLITE3_FOUND)