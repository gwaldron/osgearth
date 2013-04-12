
SET(TARGET_SRC
    TileSourceBing.cpp
)

# headers to show in IDE
SET(TARGET_H
)

SETUP_PLUGIN(osgearth_bing)


# to install public driver includes:
SET(LIB_NAME bing)
SET(LIB_PUBLIC_HEADERS "")
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
