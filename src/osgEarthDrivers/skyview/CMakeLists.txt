SET(TARGET_H
    SkyViewOptions
)
SET(TARGET_SRC 
    SkyViewTileSource.cpp
)

SETUP_PLUGIN(osgearth_skyview)


# to install public driver includes:
SET(LIB_NAME skyview)
SET(LIB_PUBLIC_HEADERS SkyViewOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

