
SET(TARGET_SRC
    BingTileSource.cpp
)

# headers to show in IDE
SET(TARGET_H
    BingOptions
)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthSymbology)

SETUP_PLUGIN(osgearth_bing)


# to install public driver includes:
SET(LIB_NAME bing)
SET(LIB_PUBLIC_HEADERS BingOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
